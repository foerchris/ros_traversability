cmake_minimum_required(VERSION 2.8.3)
project(simulations_kontrolle)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

list (APPEND CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
include (CommonMacros)

require_custom_package (OpenCV 3)



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
  std_msgs
  geometry_msgs
  sensor_msgs
  flipper_control
  robot_navigation
)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)
find_package(flipper_control REQUIRED)
find_package(robot_navigation REQUIRED)

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
  ${flipper_controlINCLUDE_DIRS}
  ${robot_navigationINCLUDE_DIRS}
)




add_executable(simulation_flipper_controle_node
    src/simulation_flipper_controle_node.cpp
    src/SimulationsKontrolleFlipper.cpp
    src/GetObjectInfoFromYaml.cpp
)

add_executable(simulation_navigation_controle_node
    src/simulation_navigation_controle_node.cpp
    src/SimulationsKontrolleNavigation.cpp
    src/GetObjectInfoFromYaml.cpp
    src/MazeReader.cpp
)

add_executable(get_text_file_data
    src/get_text_file_data.cpp
    src/MazeReader.cpp

)



target_link_libraries(simulation_flipper_controle_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)


target_link_libraries(simulation_navigation_controle_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

target_link_libraries(get_text_file_data
  ${catkin_LIBRARIES}
)
