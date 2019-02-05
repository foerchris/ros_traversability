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

void planeCallback (const sensor_msgs::Imu::ConstPtr& imu_ptr)
{
	static_transformStamped.transform.rotation.x = imu_ptr->orientation.x;
	static_transformStamped.transform.rotation.y = imu_ptr->orientation.y;
	static_transformStamped.transform.rotation.z = imu_ptr->orientation.z;
	static_transformStamped.transform.rotation.w = imu_ptr->orientation.w;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "plane_tf_broadcaster");

	if(strcmp(argv[1],"world")==0)
	{
		ROS_ERROR("Your static turtle name cannot be 'world'");
		return -1;
	}
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;

	ros::NodeHandle nodeHandle;
	ros::Rate rate=100;

	tf_prefix = "//GETjag";
	tf_prefix = ros::this_node::getNamespace();

	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	ros::Subscriber imuSub;
	imuSub = nodeHandle.subscribe < sensor_msgs::Imu > ("fitted_plane_quaternion", 1, planeCallback);

	destination_frame = tf_prefix + "/" + argv[1];
	original_frame = tf_prefix + "/" + argv[2];

	static_transformStamped.header.stamp = ros::Time::now();
	static_transformStamped.header.frame_id = original_frame;
	static_transformStamped.child_frame_id = destination_frame;

	static_transformStamped.transform.translation.x = atof(argv[3]);
	static_transformStamped.transform.translation.y = atof(argv[4]);
	static_transformStamped.transform.translation.z = atof(argv[5]);
	tf2::Quaternion quat;
	quat.setRPY(atof(argv[6]),atof(argv[7]), atof(argv[8]));
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

