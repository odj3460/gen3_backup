cmake_minimum_required(VERSION 3.0.2)
project(kinova_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  kortex_driver
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
  kortex_driver
  message_generation
  actionlib_msgs
  actionlib
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)

add_executable(kinova_move src/kinova_move.cpp)
add_dependencies(kinova_move ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinova_move ${catkin_LIBRARIES} )

add_executable(kinova_move_ft src/kinova_move_ft.cpp)
add_dependencies(kinova_move_ft ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinova_move_ft ${catkin_LIBRARIES} )

add_executable(kinova_move_admit src/kinova_move_admit.cpp)
add_dependencies(kinova_move_admit ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinova_move_admit ${catkin_LIBRARIES} )

add_executable(kinova_move_admit2 src/kinova_move_admit2.cpp)
add_dependencies(kinova_move_admit2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinova_move_admit2 ${catkin_LIBRARIES} )

add_executable(apimove_test src/apimove_test.cpp)
add_dependencies(apimove_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(apimove_test ${catkin_LIBRARIES} )

add_executable(move_low_test src/move_low_test.cpp)
add_dependencies(move_low_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_low_test ${catkin_LIBRARIES} )

add_executable(move_low_test_one src/move_low_test_one.cpp)
add_dependencies(move_low_test_one ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_low_test_one ${catkin_LIBRARIES} )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinova_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
