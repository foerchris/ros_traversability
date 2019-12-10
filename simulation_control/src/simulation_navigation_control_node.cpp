/*
 * robot_positioning.cpp
 *
 *  Created on: 10.10.2018
 *      Author: chfo
 */

/// Node for autonomous navigation scenario creation

#include <ros/ros.h>
#include "simulations_kontrolle/SimulationsKontrolleNavigation.h"
#include <signal.h>



std::string BASE_FRAME = "/base_link";
std::string MAP_FRAME = "/map";
std::string ODOM_FRAME = "/odom";
std::string gazeboMoveObjectFrame = "world";
std::string tf_prefix = "//GETjag2";


int main(int argc, char** argv) {
	ros::init(argc, argv, "simulation_navigation_controle_node");
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

	SimulationsKontrolleNavigation simulationsKontrolleNavigation(nh);

	std::this_thread::sleep_for(std::chrono::seconds(5));

	while(ros::ok())
	{
		ros::spin ();
	}

	return 0;
}
