cmake_minimum_required(VERSION 2.8.3)
project(elevation_mapper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -std=c++11")


list (APPEND CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
include (CommonMacros)

require_custom_package (OpenCV 3)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_rviz_plugin
  grid_map_visualization
  geometry_msgs
  sensor_msgs
  image_transport
  cv3_bridge
  filters
  pcl_conversions
  pcl_ros
  laser_assembler
  tf_conversions
  camera_calibration_parsers)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS   grid_map_core grid_map_ros grid_map_cv grid_map_filters grid_map_loader grid_map_msgs 	grid_map_rviz_plugin grid_map_visualization tf_conversions
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

)


## Declare a cpp executable

add_executable(elevation_mapper_node src/elevation_mapper_node.cpp src/elevationmapper.cpp)

add_executable(test_node src/test.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  elevation_mapper_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(
  test_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
