/*
 * flipper_tf_broadcaster.cpp
 *
 *  Created on: 21.12.2018
 *      Author: chfo
 */


#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>
#include <flipper_control/broadcasterConfig.h>


std::string destination_frame = "/base_link";
std::string original_frame = "/map";
std::string new_frame;

std::string tf_prefix = "GETjag";

#include <tf/transform_datatypes.h>

geometry_msgs::TransformStamped static_transformStamped;

double globalyaw = 0;

double fixed_velocity = 0;
geometry_msgs::Twist currentVelocity;
double delta_t = 1.2;
geometry_msgs::Pose deltaPose;
geometry_msgs::Pose initPose;

void odomCallback (const nav_msgs::OdometryConstPtr& odomMsg)
{
	currentVelocity = odomMsg->twist.twist;
	double theta = currentVelocity.angular.z * delta_t;
	double v_dot = currentVelocity.linear.x * delta_t;
	//double v_dot = fixed_velocity* delta_t;

	static_transformStamped.header.stamp = ros::Time::now();

	static_transformStamped.transform.translation.x = initPose.position.x +  v_dot * cos(theta) ;
	static_transformStamped.transform.translation.y = initPose.position.y + v_dot * sin(theta);
	static_transformStamped.transform.translation.z = initPose.position.z + 0;

	tf2::Quaternion quat;

	if(globalyaw==0)
	{
		quat.setRPY(0,0,theta);
	}
	else
	{
		quat.setRPY(0,0,theta-globalyaw);
	}


	static_transformStamped.transform.rotation.x = quat.getX();
	static_transformStamped.transform.rotation.y = quat.getY();
	static_transformStamped.transform.rotation.z = quat.getZ();
	static_transformStamped.transform.rotation.w = quat.getW();
}
void reconfigureCallback(flipper_control::broadcasterConfig &config, uint32_t level)
{
		  fixed_velocity =config.fixed_velocity;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "tf_broadcaster");

	if(strcmp(argv[1],"world")==0)
	{
		ROS_ERROR("Your static turtle name cannot be 'world'");
		return -1;
	}



	static tf2_ros::StaticTransformBroadcaster static_broadcaster;

	ros::NodeHandle nodeHandle;
	ros::Rate rate=100;
	ros::Subscriber odomSub = nodeHandle.subscribe<nav_msgs::Odometry> ("odom", 20, odomCallback);
	// setting up reconfigure servers
	static dynamic_reconfigure::Server<flipper_control::broadcasterConfig> server;
	static dynamic_reconfigure::Server<flipper_control::broadcasterConfig>::CallbackType f;
	f = boost::bind(&reconfigureCallback,_1, _2);
	server.setCallback(f);

	tf_prefix = "//GETjag";
	tf_prefix = ros::this_node::getNamespace();

	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);


	destination_frame = tf_prefix + "/" + argv[1];
	original_frame = tf_prefix + "/" + argv[2];

	static_transformStamped.header.stamp = ros::Time::now();
	static_transformStamped.header.frame_id = original_frame;
	static_transformStamped.child_frame_id = destination_frame;

	static_transformStamped.transform.translation.x = atof(argv[3]);
	static_transformStamped.transform.translation.y = atof(argv[4]);
	static_transformStamped.transform.translation.z = atof(argv[5]);
	initPose.position.x = atof(argv[3]);
	initPose.position.y = atof(argv[4]);
	initPose.position.z = atof(argv[5]);

	globalyaw = atof(argv[8]);
	tf2::Quaternion quat;
	quat.setRPY(0,0, globalyaw);
	static_transformStamped.transform.rotation.x = quat.x();
	static_transformStamped.transform.rotation.y = quat.y();
	static_transformStamped.transform.rotation.z = quat.z();
	static_transformStamped.transform.rotation.w = quat.w();


	ROS_INFO("Spinning until killed publishing %s to world", new_frame.c_str());
	while(ros::ok())
	{
		static_broadcaster.sendTransform(static_transformStamped);
		ros::spinOnce();
		rate.sleep();

	}
	return 0;
};

