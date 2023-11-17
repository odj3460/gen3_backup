cmake_minimum_required(VERSION 3.0.2)
project(dual_interface)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

option(USE_CONAN "Use the Conan package manager to automatically fetch the Kortex API" ON)

if(NOT CONAN_TARGET_PLATFORM)
  set(CONAN_TARGET_PLATFORM "x86")
endif()
message("CONAN_TARGET_PLATFORM is ${CONAN_TARGET_PLATFORM}")

# Explicitely specify the build type in case the user did not do it
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs controller_manager_msgs message_generation actionlib control_msgs urdf)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(${PROJECT_SOURCE_DIR}/thirdParty/cxxopts/)

## declare a catkin package
catkin_package(CATKIN_DEPENDS actionlib_msgs)

function(get_closest_build_type VAR_OUT)
  if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(${VAR_OUT} "Debug" PARENT_SCOPE)
    return()
  endif()
  string(TOUPPER ${CMAKE_BUILD_TYPE} BTYPE)
  if("${BTYPE}" STREQUAL "DEBUG")
    set(${VAR_OUT} "Debug" PARENT_SCOPE)
  else()
    set(${VAR_OUT} "Release" PARENT_SCOPE)
  endif()
endfunction()

if(USE_CONAN)
  # Include conan.cmake module and download Kortex API from artifactory
  include(${PROJECT_SOURCE_DIR}/cmake/conan.cmake)
  conan_check(REQUIRED)
  conan_add_remote(NAME kinova_public
                  URL https://artifactory.kinovaapps.com/artifactory/api/conan/conan-public)
  get_closest_build_type(CONAN_BUILD_TYPE)
  if("${CONAN_TARGET_PLATFORM}" STREQUAL "x86")
    conan_cmake_run(CONANFILE conanfile.py
                    UPDATE
                    BASIC_SETUP CMAKE_TARGETS
                    NO_OUTPUT_DIRS
                    SETTINGS kortex_api_cpp:compiler=gcc kortex_api_cpp:compiler.version=5 kortex_api_cpp:compiler.libcxx=libstdc++11 kortex_api_cpp:build_type=${CONAN_BUILD_TYPE})
  elseif("${CONAN_TARGET_PLATFORM}" STREQUAL "artik710")
    conan_cmake_run(CONANFILE conanfile.py
                    UPDATE
                    BASIC_SETUP CMAKE_TARGETS
                    NO_OUTPUT_DIRS
                    SETTINGS kortex_api_cpp:arch=armv7 kortex_api_cpp:compiler=gcc kortex_api_cpp:compiler.version=5 kortex_api_cpp:compiler.libcxx=libstdc++11 kortex_api_cpp:build_type=${CONAN_BUILD_TYPE}
                    ENV TARGET=artik710)
  elseif("${CONAN_TARGET_PLATFORM}" STREQUAL "imx6")
    conan_cmake_run(CONANFILE conanfile.py
                    UPDATE
                    BASIC_SETUP CMAKE_TARGETS
                    NO_OUTPUT_DIRS
                    SETTINGS kortex_api_cpp:arch=armv7 kortex_api_cpp:compiler=gcc kortex_api_cpp:compiler.version=6.4 kortex_api_cpp:compiler.libcxx=libstdc++11 kortex_api_cpp:build_type=${CONAN_BUILD_TYPE}
                    ENV TARGET=imx6)
  elseif("${CONAN_TARGET_PLATFORM}" STREQUAL "jetson")
    conan_cmake_run(CONANFILE conanfile.py
                    UPDATE
                    BASIC_SETUP CMAKE_TARGETS
                    NO_OUTPUT_DIRS
                    SETTINGS kortex_api_cpp:arch=armv7 kortex_api_cpp:compiler=gcc kortex_api_cpp:compiler.version=7 kortex_api_cpp:compiler.libcxx=libstdc++11 kortex_api_cpp:build_type=${CONAN_BUILD_TYPE}
                    ENV TARGET=jetson)
  endif()
  
  link_libraries(${CONAN_LIBS})
  
endif()



include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${Boost_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/include)
include_directories(include ${PROJECT_SOURCE_DIR}/include/dual_interface)



if(NOT USE_CONAN)
  include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
  include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
  include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
  include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)
  include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include)
  link_directories(${PROJECT_SOURCE_DIR}/../kortex_api/lib/release)
endif()


add_executable(dual_node 
	src/dual_node.cpp

	)

