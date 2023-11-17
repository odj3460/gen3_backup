cmake_minimum_required(VERSION 3.5)

project(dualapicontrol VERSION 2.3.0 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR   ON)
set(CMAKE_VERBOSE_MAKEFILE      ON)
set(CMAKE_COLOR_MAKEFILE        ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


option(USE_CONAN "Use the Conan package manager to automatically fetch the Kortex API" ON)
option(DOWNLOAD_API "Automatically download the API if conan is not used" ON)

# Activate C++ 11
set (CMAKE_CXX_STANDARD 11)


macro(configure_msvc_runtime)

    # Default to statically-linked runtime.
    if("${MSVC_RUNTIME}" STREQUAL "")
      set(MSVC_RUNTIME "static")
    endif()

    # Set compiler options.
    set(variables
      CMAKE_C_FLAGS_DEBUG
      CMAKE_C_FLAGS_MINSIZEREL
      CMAKE_C_FLAGS_RELEASE
      CMAKE_C_FLAGS_RELWITHDEBINFO
      CMAKE_CXX_FLAGS_DEBUG
      CMAKE_CXX_FLAGS_MINSIZEREL
      CMAKE_CXX_FLAGS_RELEASE
      CMAKE_CXX_FLAGS_RELWITHDEBINFO
    )
    
    if(${MSVC_RUNTIME} STREQUAL "static")
      message(STATUS
        "MSVC -> forcing use of statically-linked runtime."
      )
      foreach(variable ${variables})
        if(${variable} MATCHES "/MD")
          string(REGEX REPLACE "/MD" "/MT" ${variable} "${${variable}}")
        endif()
      endforeach()
    
    else()
      message(STATUS
        "MSVC -> forcing use of dynamically-linked runtime."
      )
      foreach(variable ${variables})
        if(${variable} MATCHES "/MT")
          string(REGEX REPLACE "/MT" "/MD" ${variable} "${${variable}}")
        endif()
      endforeach()

    endif()

endmacro()


if(MSVC)
  configure_msvc_runtime()
else()
  include_directories(${PROJECT_SOURCE_DIR}/thirdParty/cxxopts/)
  add_compile_options(-Wall)
  add_compile_options(-Wno-reorder)

endif()

if(UNIX)
  add_definitions(-D_OS_UNIX)
elseif(WIN32)
  add_definitions(-D_OS_WINDOWS -DNOMINMAX)
  if(MSVC)
     add_compile_options(/bigobj)
  endif()
endif()

if(USE_CONAN)

  include(${PROJECT_SOURCE_DIR}/thirdParty/conan.cmake)

  conan_check(REQUIRED)
  conan_add_remote(
    NAME kinova_public 
    URL https://artifactory.kinovaapps.com/artifactory/api/conan/conan-public)

  if(UNIX)
    conan_cmake_run(REQUIRES kortex_api_cpp/2.3.0-r.34@kortex/stable
                    SETTINGS kortex_api_cpp:compiler=gcc
                    SETTINGS kortex_api_cpp:compiler.version=5
                    SETTINGS compiler.libcxx=libstdc++11
                    PROFILE_AUTO build_type
                    BASIC_SETUP
                    UPDATE)
  elseif(WIN32)
    if(MSVC)

      _get_msvc_ide_version(_VISUAL_VERSION)

      # TODO Conan artifacts should not require to specify a target to download
      if (_VISUAL_VERSION EQUAL 14)
        set(kortex_api_cpp_target "msvc-2015")
      elseif(_VISUAL_VERSION EQUAL 15)
        set(kortex_api_cpp_target "msvc-2017")
      elseif(_VISUAL_VERSION EQUAL 16)
        set(kortex_api_cpp_target "msvc-2019")
      endif()

      conan_cmake_run(REQUIRES kortex_api_cpp/2.3.0-r.34@kortex/stable
                      PROFILE_AUTO build_type
                      BASIC_SETUP
                      UPDATE)
      
    else()
      conan_cmake_run(REQUIRES kortex_api_cpp/2.3.0-r.34@kortex/stable
                      SETTINGS kortex_api_cpp:compiler=gcc
                      SETTINGS kortex_api_cpp:compiler.version=5
                      SETTINGS compiler.libcxx=libstdc++11
                      PROFILE_AUTO build_type
                      BASIC_SETUP
                      UPDATE)
    endif()
  endif()

  link_libraries(${CONAN_LIBS})

else() # Not using Conan

  # Setup Kortex Api Path
  if(NOT KORTEX_SUB_DIR)
    set(KORTEX_SUB_DIR "")
  else()
    set(KORTEX_SUB_DIR "${KORTEX_SUB_DIR}/")
  endif()

  set(KORTEX_DIR "${PROJECT_SOURCE_DIR}/kortex_api/${KORTEX_SUB_DIR}")

  if(CMAKE_BUILD_TYPE EQUAL "Debug")
    set(KORTEX_LIB_SUBDIR "debug")
  else()
    set(KORTEX_LIB_SUBDIR "release")
  endif()

  # Download the API
  if(DOWNLOAD_API)
    if(UNIX)
      execute_process(COMMAND ./download_kortex_api.sh ${KORTEX_SUB_DIR}
        WORKING_DIRECTORY ../scripts
        RESULT_VARIABLE DOWNLOAD_API_RESULT
        OUTPUT_VARIABLE DOWNLOAD_API_OUTPUT)
      if(NOT DOWNLOAD_API_RESULT EQUAL 0)
        message("Kortex API was not downloaded prior to running CMake.")
        message(FATAL_ERROR ${DOWNLOAD_API_OUTPUT})
      endif()
    elseif(WIN32)
      execute_process(COMMAND ./download_kortex_api.bat ${KORTEX_SUB_DIR}
        WORKING_DIRECTORY ../scripts
        RESULT_VARIABLE DOWNLOAD_API_RESULT
        OUTPUT_VARIABLE DOWNLOAD_API_OUTPUT)
      if(NOT DOWNLOAD_API_RESULT EQUAL 0)
        message("Kortex API was not downloaded prior to running CMake.")
        message(FATAL_ERROR ${DOWNLOAD_API_OUTPUT})
      endif()
    endif()
  endif()
  if(UNIX)
    link_libraries(${KORTEX_DIR}lib/${KORTEX_LIB_SUBDIR}/libKortexApiCpp.a)
  elseif(WIN32)
    link_libraries(${KORTEX_DIR}lib/${KORTEX_LIB_SUBDIR}/KortexApiCpp.lib)
  endif()

  # Add Include Directories
  include_directories(${KORTEX_DIR}include)
  include_directories(${KORTEX_DIR}include/client)
  include_directories(${KORTEX_DIR}include/common)
  include_directories(${KORTEX_DIR}include/messages)
  include_directories(${KORTEX_DIR}include/client_stubs)

endif()

# link other libs
link_libraries(pthread)

find_package(PkgConfig REQUIRED)
pkg_check_modules(orocos_kdl REQUIRED orocos_kdl)
find_package(kdl_parser)
find_package(Boost COMPONENTS system filesystem REQUIRED)

include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)


# Create executable for each example
# Look for examples under folders
#file(GLOB EXE_LIST RELATIVE ${PROJECT_SOURCE_DIR} "src/[0-9]*-*/[0-9]*.cpp")
file(GLOB EXE_LIST RELATIVE ${PROJECT_SOURCE_DIR} "src/controllers/*.cpp")
foreach ( SRC_FILE ${EXE_LIST} )

  string(REPLACE ".cpp" "" TARGET_EXE_NAME ${SRC_FILE})
  string(REPLACE "/" "_" TARGET_EXE_NAME ${TARGET_EXE_NAME})
  
  MESSAGE("creating TARGET_EXE_NAME: '${TARGET_EXE_NAME}'")

  add_executable(${TARGET_EXE_NAME} ${SRC_FILE} 
  src/util/utilities.cpp 
  src/util/LowPassFilter.cpp
  src/util/LowPassFilter_2.cpp
  src/util/LowPassFilter_3.cpp
  src/util/kinova_mediator_L.cpp
  src/util/kinova_mediator_R.cpp
  src/util/constants.cpp
  #src/util/dynamics_controller.cpp
  #src/util/solver_recursive_newton_euler.cpp
  #src/util/fk_vereshchagin.cpp
  #src/util/safety_controller.cpp
  #src/util/finite_state_machine.cpp
  #src/util/motion_profile.cpp
  
  src/util/chainexternalwrenchestimator.cpp
  src/util/chainidsolver_recursive_newton_euler.cpp
  
  
  )
  
  target_link_libraries(${TARGET_EXE_NAME}
    ${orocos_kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
	)
  
endforeach()


