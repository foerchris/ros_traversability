cmake_minimum_required(VERSION 2.8.3)
project(robot_navigation)

add_compile_options ("-Wall" "-std=c++11")

list (APPEND CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
include (CommonMacros)

require_custom_package (PCL 1)

require_custom_package (OpenCV 3)

require_custom_package (ompl 1)


find_package(catkin REQUIRED COMPONENTS
	dynamic_reconfigure
	geometry_msgs
  	roscpp
  	rospy
  	sensor_msgs
  	control_msgs
  	tf
  	std_msgs
  	message_filters
  	message_generation
  	cv3_bridge
  	image_transport
  	pcl_ros
  	get_std_msgs
	get_vis_msgs
	laser_geometry
	visualization_msgs
	dynamixel_msgs
)


## Generate messages in the 'msg' folder
# add_message_files(FILES
#   RobotBehaviorsConfig.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
# FILES
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
## Generate added messages and services with any dependencies listed here
#generate_messages(DEPENDENCIES
#	std_msgs
#)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/Step.cfg
   cfg/StairBehavior.cfg
   cfg/TrackingControl.cfg
   cfg/StepBehavior.cfg
)
include_directories(
    include
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${OMPL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
)

catkin_package(
      INCLUDE_DIRS include
	   LIBRARIES robot_navigation
	  CATKIN_DEPENDS message_runtime std_msgs dynamic_reconfigure geometry_msgs roscpp sensor_msgs tf get_std_msgs get_vis_msgs
	#  DEPENDS system_lib
)

add_library(robot_navigation  src/TrackingControl.cpp)


target_link_libraries(robot_navigation
                        ${catkin_LIBRARIES}
)


add_executable(track_movement
				src/track_movement.cpp
                )

add_executable(robot_navigation_server
				src/robot_navigation_server.cpp
				src/RobotControl.cpp
				src/TrackingControl.cpp
				src/YamlToMsgConverter.cpp
				src/StairBehavior.cpp
				src/StepBehavior.cpp
				src/SnakeMode.cpp
				)


add_executable(auto_robot_navigation_server
				src/auto_robot_navigation_server.cpp
				src/RobotDrive.cpp
				src/ContourPathPlanner.cpp
				src/TrackingControl.cpp
				)

add_executable(flipper_control_server
				src/flipper_control_server.cpp
				src/FlipperControl.cpp
				)

add_executable(flipper_tf_broadcaster
				src/flipper_tf_broadcaster.cpp
				)

target_link_libraries(robot_navigation_server
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OMPL_LIBRARIES}
  yaml-cpp
  ${catkin_LIBRARIES}
)
target_link_libraries(auto_robot_navigation_server
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

target_link_libraries(track_movement
  ${catkin_LIBRARIES}
)

target_link_libraries(flipper_control_server
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(flipper_tf_broadcaster
  ${catkin_LIBRARIES}
)

add_dependencies(robot_navigation_server ${PROJECT_NAME}_gencpp)
add_dependencies(robot_navigation_server ${PROJECT_NAME}_gencfg)

add_dependencies(auto_robot_navigation_server ${PROJECT_NAME}_gencpp)
add_dependencies(auto_robot_navigation_server ${PROJECT_NAME}_gencfg)


install(TARGETS robot_navigation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


