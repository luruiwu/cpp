cmake_minimum_required(VERSION 2.8.3)
project(imi_nav_v2)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
geometry_msgs
nav_msgs
roscpp
rospy
std_msgs
sensor_msgs
move_base_msgs
tf
cv_bridge
genmsg
actionlib_msgs
actionlib
eigen_conversions
)


find_package(OpenCV 3.3.1 REQUIRED)
find_package(Eigen3 REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imi_nav_v2
  CATKIN_DEPENDS roscpp rospy std_msgs tf cv_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########
#message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message("EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
MESSAGE("EIGEN3_LIBS: ${EIGEN3_LIBS}")
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(
${OpenCV_INCLUDE_DIRS}
)
include_directories(
${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -Wno-deprecated")

## Declare a cpp library
add_library(imi_nav_v2
  src/${PROJECT_NAME}/imi_nav.cpp
  src/${PROJECT_NAME}/imi_coverage_path_planning.cpp
  src/${PROJECT_NAME}/imi_nav_bcd.cpp
  src/building_navigation/a_star_pathplanner.cpp
  src/building_navigation/concorde_TSP.cpp
  src/building_navigation/genetic_TSP.cpp
  src/building_navigation/nearest_neighbor_TSP.cpp
  src/building_navigation/node.cpp
  src/room_exploration/boustrophedon.cpp
  src/room_exploration/fov_to_robot_mapper.cpp
  src/room_exploration/map_accessibility_analysis.cpp
  src/room_exploration/room_rotator.cpp
  src/room_exploration/meanshift2d.cpp
  src/room_exploration/cell_decomposition.cpp
)

target_link_libraries(imi_nav_v2
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(imi_nav_node_v2 src/imi_nav_node.cpp)
add_executable(imi_nav_bcd src/imi_nav_bcd.cpp)
add_executable(imi_coverage_path_planning src/imi_coverage_path_planning.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(imi_nav_node_v2
  ${catkin_LIBRARIES}
  imi_nav_v2
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
)

target_link_libraries(imi_nav_bcd
  ${catkin_LIBRARIES}
  imi_nav_v2
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
)

target_link_libraries(imi_coverage_path_planning
  ${catkin_LIBRARIES}
  imi_nav_v2
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
 install(TARGETS imi_nav_v2 imi_nav_node_v2
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

### Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

install(DIRECTORY launch cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
