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
   cfg/broadcaster.cfg

)
include_directories(
    include
	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
)

catkin_package(
      INCLUDE_DIRS include
      LIBRARIES flipper_control
	  CATKIN_DEPENDS message_runtime std_msgs dynamic_reconfigure geometry_msgs roscpp sensor_msgs tf get_std_msgs get_vis_msgs
	#  DEPENDS system_lib
)

add_library(flipper_control  src/GetContactPoints.cpp)


target_link_libraries(flipper_control
                        ${OpenCV_LIBRARIES}
                        ${catkin_LIBRARIES}
)

add_executable(flipper_control_server
				src/flipper_control_server.cpp
				src/FlipperControl.cpp
				src/GetContactPoints.cpp
				src/CalcFlipperAngles.cpp
				src/FitPlane.cpp
				src/ClcNESM.cpp
				)

add_executable(tf_broadcaster
				src/tf_broadcaster.cpp
				)

add_executable(flipper_tf_broadcaster
				src/flipper_tf_broadcaster.cpp
				)

add_executable(plane_tf_broadcaster
				src/plane_tf_broadcaster.cpp
				)

target_link_libraries(flipper_control_server
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(tf_broadcaster
  ${catkin_LIBRARIES}
)

target_link_libraries(flipper_tf_broadcaster
  ${catkin_LIBRARIES}
)


target_link_libraries(plane_tf_broadcaster
  ${catkin_LIBRARIES}
)



add_dependencies(flipper_control_server ${PROJECT_NAME}_gencpp)
add_dependencies(flipper_control_server ${PROJECT_NAME}_gencfg)

add_dependencies(tf_broadcaster ${PROJECT_NAME}_gencpp)
add_dependencies(tf_broadcaster ${PROJECT_NAME}_gencfg)



install(TARGETS flipper_control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

