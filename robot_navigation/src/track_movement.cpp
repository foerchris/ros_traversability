/*
 * object_follower_server.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: chfo
 */
#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"

#include <nav_msgs/Odometry.h>



void odomCallback (const nav_msgs::OdometryConstPtr& odomMsg)
{

	/*double currentLinearVelocity = odomMsg->twist.twist.linear.x;
	double currentAngularVelocity = odomMsg->twist.twist.angular.z;
	odomMsg->*/
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_robot_navigation_server");
	ros::NodeHandle nh;

    ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry> ("odom", 10, odomCallback);

	//ros::Subscriber CloudListener = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("xtion/depth/points", 10, cloudCallback);

	ros::spin();
	return 0;
}



