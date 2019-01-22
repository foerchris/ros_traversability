cmake_minimum_required(VERSION 2.8.3)
project(flipper_control)

add_compile_options ("-Wall" "-std=c++11")

list (APPEND CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
include (CommonMacros)

require_custom_package (OpenCV 3)


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
   cfg/FlipperControl.cfg
)
include_directories(
	${OpenCV_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/src/include
	${EIGEN3_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
)

catkin_package(
	 INCLUDE_DIRS src/include
	  CATKIN_DEPENDS message_runtime std_msgs dynamic_reconfigure geometry_msgs roscpp sensor_msgs tf get_std_msgs get_vis_msgs
	#  DEPENDS system_lib
)


add_executable(flipper_control_server
				src/flipper_control_server.cpp
				src/FlipperControl.cpp
				src/GetContactPoints.cpp
				src/CalcFlipperAngles.cpp
				src/FitPlane.cpp
				)

add_executable(flipper_tf_broadcaster
				src/flipper_tf_broadcaster.cpp
				)

add_executable(desired_pose_tf_broadcaster
				src/desired_pose_tf_broadcaster.cpp
				)

target_link_libraries(flipper_control_server
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(flipper_tf_broadcaster
  ${catkin_LIBRARIES}
)
target_link_libraries(desired_pose_tf_broadcaster
  ${catkin_LIBRARIES}
)
add_dependencies(flipper_control_server ${PROJECT_NAME}_gencpp)
add_dependencies(flipper_control_server ${PROJECT_NAME}_gencfg)



