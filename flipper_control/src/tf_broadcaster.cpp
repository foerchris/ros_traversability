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


std::string destination_frame = "/base_link";
std::string original_frame = "/map";
std::string new_frame;

std::string tf_prefix = "GETjag";

#include <tf/transform_datatypes.h>

geometry_msgs::TransformStamped static_transformStamped;

double globalyaw = 0;

/*
geometry_msgs::Pose clcDeltaPose(const geometry_msgs::Twist& velocitiy_robot, const double& delta_t)
{
	//double theta = velocitiy_robot.angular.z*delta_t;
	double theta = 0;
	//double v_dot = velocitiy_robot.linear.x * delta_t;
	double v_dot = 2*0.08;
	deltaPose.position.x = v_dot * cos(theta);
	deltaPose.position.y = v_dot * sin(theta);

	deltaPose.position.z = 0;

	tf2::Quaternion quat;

	quat.setRPY(0,0,theta);
	deltaPose.orientation.x = quat.getX();
	deltaPose.orientation.y = quat.getY();
	deltaPose.orientation.z = quat.getZ();
	deltaPose.orientation.w = quat.getW();

	return deltaPose;
}*/

geometry_msgs::Twist currentVelocity;
double delta_t = 0.8;
geometry_msgs::Pose deltaPose;
geometry_msgs::Pose initPose;

void odomCallback (const nav_msgs::OdometryConstPtr& odomMsg)
{
	currentVelocity = odomMsg->twist.twist;
	double theta = currentVelocity.angular.z * delta_t;
	double v_dot = currentVelocity.linear.x * delta_t;

	static_transformStamped.header.stamp = ros::Time::now();

	static_transformStamped.transform.translation.x = initPose.position.x +  v_dot * cos(theta) ;
	static_transformStamped.transform.translation.y = initPose.position.y + v_dot * sin(theta);
	static_transformStamped.transform.translation.z = initPose.position.y + 0;

	tf2::Quaternion quat;
	if(globalyaw == 0)
	{
		quat.setRPY(0,0,theta);
	}
	static_transformStamped.transform.rotation.x = quat.getX();
	static_transformStamped.transform.rotation.y = quat.getY();
	static_transformStamped.transform.rotation.z = quat.getZ();
	static_transformStamped.transform.rotation.w = quat.getW();
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "flipper_tf_broadcaster");

	if(strcmp(argv[1],"world")==0)
	{
		ROS_ERROR("Your static turtle name cannot be 'world'");
		return -1;
	}



	static tf2_ros::StaticTransformBroadcaster static_broadcaster;

	ros::NodeHandle nodeHandle;
	ros::Rate rate=100;
	ros::Subscriber odomSub = nodeHandle.subscribe<nav_msgs::Odometry> ("odom", 1, odomCallback);

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

