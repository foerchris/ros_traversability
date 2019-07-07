/*
 * robot_positioning.cpp
 *
 *  Created on: 10.10.2018
 *      Author: chfo
 */

#include <ros/ros.h>
#include "traversability_estimation/GazebObjectControl.h"



std::string BASE_FRAME = "/base_link";
std::string MAP_FRAME = "/map";
std::string ODOM_FRAME = "/odom";
std::string gazeboMoveObjectFrame = "world";
std::string tf_prefix = "//GETjag2";



int main(int argc, char** argv) {
	ros::init(argc, argv, "gazebo_control_node");
	ros::NodeHandle nh;
	tf_prefix = ros::this_node::getNamespace();

	if(tf_prefix == "/")
	{
		tf_prefix = "//GETjag1";
	}
	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	std::istringstream iss (tf_prefix.substr(6, tf_prefix.size()));

	int robot_number = 1;
	iss >> robot_number;

	BASE_FRAME = tf_prefix + BASE_FRAME;
	MAP_FRAME = tf_prefix + MAP_FRAME;
	ODOM_FRAME = tf_prefix + ODOM_FRAME;
	ros::Rate rate (10);
	ros::Rate rate2sec (0.5);


	GazebObjectControl gazebObjectControl(nh);

	geometry_msgs::Pose startPose;
	startPose.position.x = 18;
	startPose.position.y = 0;
	startPose.position.z = 0;

	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0, 0, 0);

	startPose.orientation.x = myQuaternion.x();
	startPose.orientation.y = myQuaternion.y();
	startPose.orientation.z = myQuaternion.z();
	startPose.orientation.w = myQuaternion.w();

	std::string resetParam = "End_of_episode";
	bool reset = true;

	while(ros::ok())
	{
		if(reset == true)
		{
			nh.setParam(resetParam,false);
			reset = false;

			// set all getjag to a zero pose
			gazebObjectControl.setRobotZeroPose();

			//reset obstacles
			gazebObjectControl.destroyWorld();

			gazebObjectControl.generateWorld(2,4);


			//std::this_thread::sleep_for(std::chrono::seconds(1));

			nh.setParam("reset_elevation_map",true);
			std::this_thread::sleep_for(std::chrono::seconds(4));

			nh.setParam("Ready_to_Start_DRL_Agent",true);

		}

		if(nh.hasParam(resetParam))
		{
		nh.getParam(resetParam,reset);
		}

		gazebObjectControl.publischGoal(startPose);
		ros::spinOnce ();
		rate.sleep();
	}

	return 0;
}
