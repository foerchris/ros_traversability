/*
 * object_follower_server.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: chfo
 */
#include <RobotDrive.h>



int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_robot_navigation_server");
	ros::NodeHandle nh;
	RobotDrive robotControl(nh);
	ros::spin();
	return 0;
}



