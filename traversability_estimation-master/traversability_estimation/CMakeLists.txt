cmake_minimum_required(VERSION 2.8.3)
project(traversability_estimation)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
list (APPEND CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
include (CommonMacros)

require_custom_package (OpenCV 3)

require_custom_package (ompl 1)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  grid_map_ros
  grid_map_core
  grid_map_msgs
  grid_map_filters
  roscpp
  cv3_bridge
  image_transport
  tf
  tf_conversions
  traversability_estimation_filters
  traversability_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  flipper_control
  robot_navigation
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(kindr QUIET)
find_package(PkgConfig REQUIRED)
find_package(flipper_control REQUIRED)
find_package(robot_navigation REQUIRED)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  pkg_check_modules(kindr kindr REQUIRED)
endif()


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
  INCLUDE_DIRS
    include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    grid_map_ros
    grid_map_core
    grid_map_msgs
    grid_map_filters
    roscpp
    tf
    tf_conversions
    traversability_estimation_filters
    traversability_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
    flipper_control
    robot_navigation
#  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${kindr_INCLUDE_DIRS}
  ${flipper_controlINCLUDE_DIRS}
  ${robot_navigationINCLUDE_DIRS}
)

## Declare a cpp library
add_library(
  ${PROJECT_NAME}
  src/TraversabilityMap.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
   ${OMPL_LIBRARIES}
)

## Declare a cpp executable
add_executable(
  ${PROJECT_NAME}_node
  src/traversability_estimation_node.cpp
  src/TraversabilityEstimation.cpp
  src/RobotPathPlanner.cpp
  src/OMPLPlannerWrapper.cpp
)

add_executable(robot_positioning_node
    src/robot_positioning_node.cpp
    src/MazeReader.cpp
)

add_executable(gazebo_control_node
    src/gazebo_control_node.cpp
    src/GazebObjectControl.cpp
    src/GetObjectInfoFromYaml.cpp
)

add_executable(gazebo_random_control_node
    src/gazebo_random_control_node.cpp
    src/GazebRandomObjectControl.cpp
    src/GetObjectInfoFromYaml.cpp
    src/MazeReader.cpp
)
add_executable(robot_positioning_node_random
    src/robot_positioning_node_random.cpp
    src/MazeReader.cpp
)

add_executable(get_text_file_data
    src/get_text_file_data.cpp
    src/MazeReader.cpp

)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(robot_positioning_node
  ${catkin_LIBRARIES}
)

target_link_libraries(gazebo_control_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)


target_link_libraries(gazebo_random_control_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)


target_link_libraries(robot_positioning_node_random
  ${catkin_LIBRARIES}
)

target_link_libraries(get_text_file_data
  ${catkin_LIBRARIES}
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
