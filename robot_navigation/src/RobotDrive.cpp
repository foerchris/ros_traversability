/*
 * approachPoint.cpp
 *
 *  Created on: Mar 28, 2018
 *      Author: chfo
 */
#include "RobotDrive.h"
#include "tf/transform_datatypes.h"

using namespace std;
RobotDrive::RobotDrive(ros::NodeHandle& nodeHandle)
	: nodeHandle_(nodeHandle)
{

	BASE_FRAME = "/base_link";
	MAP_FRAME = "/map";
	ODOM_FRAME = "/odom";
	tf_prefix = "//GETjag1";

	tf_prefix = ros::this_node::getNamespace();
	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	istringstream iss (tf_prefix.substr(6, tf_prefix.size()));
	int robot_number = 1;
	iss >> robot_number;

	BASE_FRAME = tf_prefix + BASE_FRAME;
	MAP_FRAME = tf_prefix + MAP_FRAME;
	ODOM_FRAME = tf_prefix + ODOM_FRAME;

	goalSub = nodeHandle_.subscribe<nav_msgs::Odometry>("goal_pose",1,boost::bind (&RobotDrive::getGoalPose, this, _1));

	selectRegion  = nodeHandle_.advertiseService("clc_path_to_goal", &RobotDrive::clcPathSrv, this);

	tfListener = unique_ptr<tf::TransformListener> (new tf::TransformListener);
	drive = false;
}
RobotDrive::~RobotDrive()
{

}


void RobotDrive::getGoalPose(const nav_msgs::Odometry::ConstPtr& goalPoseMsg)
{


	contourPathPlanner.forwardBackwarMode(true,1,1);


	geometry_msgs::Pose goal;
	goal.position.x = goalPoseMsg->pose.pose.position.x;
	goal.position.y = goalPoseMsg->pose.pose.position.y;
	goal.position.z = goalPoseMsg->pose.pose.position.z;

	goal.orientation.x = goalPoseMsg->pose.pose.orientation.x;
	goal.orientation.y = goalPoseMsg->pose.pose.orientation.y;
	goal.orientation.z = goalPoseMsg->pose.pose.orientation.z;
	goal.orientation.w = goalPoseMsg->pose.pose.orientation.w;

	pose Pose;

	Pose.x = goal.position.x;
	Pose.y = goal.position.y;
	Pose.z = goal.position.z;

	tf::Quaternion quat;
	quat.setX(goal.orientation.x);
	quat.setY(goal.orientation.y);
	quat.setZ(goal.orientation.z);
	quat.setW(goal.orientation.w);

	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	Pose.roll = roll;
	Pose.pitch = pitch;
	Pose.yaw = yaw;

	pathPoses.push_back(Pose);

	contourPathPlanner.followPath(ReferencePathControl,drive);
}


geometry_msgs::Pose RobotDrive::tfTransform(const geometry_msgs::Pose& pose,const string& destination_frame,const string& original_frame)
{
	// TF transformation of the Point which is nearest to the robot
	const ros::Time& scanTimeStamp = ros::Time (0);

    try
	{
		tfListener->waitForTransform (destination_frame, original_frame, scanTimeStamp, ros::Duration (3.0));
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}

	tf::Quaternion quat;
	quat.setX(pose.orientation.x);
	quat.setY(pose.orientation.y);
	quat.setZ(pose.orientation.z);
	quat.setW(pose.orientation.w);


	tf::Vector3 origin;

	origin.setX(pose.position.x);
	origin.setY(pose.position.y);
	origin.setZ(pose.position.z);

	tf::StampedTransform transform;

	tf::Stamped<tf::Pose> transPose (tf::Pose( quat,origin), scanTimeStamp, original_frame);

	tf::Stamped<tf::Pose> poseTransformed;

	try
	{
		tfListener->transformPose(destination_frame, scanTimeStamp, transPose, original_frame, poseTransformed);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR(" Point xyz %s", ex.what ());
	}

	origin=poseTransformed.getOrigin();
	quat=poseTransformed.getRotation();

	geometry_msgs::Pose returnPose;
	returnPose.position.x=origin.getX();
	returnPose.position.y=origin.getY();
	returnPose.position.z=origin.getZ();
	returnPose.orientation.x = quat.x();
	returnPose.orientation.y = quat.y();
	returnPose.orientation.z = quat.z();
	returnPose.orientation.w = quat.w();

	return returnPose;
}
void RobotDrive::clcPath(double planningRadius, vector<pose> Poses)
{
	contourPathPlanner.setPathPoints(Poses);
	contourPathPlanner.clcPathToSinglePoint(planningRadius);
	drive = true;
}
bool RobotDrive::clcPathSrv(std_srvs::Empty::Request &req,
		std_srvs::Empty::Response &res)
{
	clcPath(0.5, pathPoses);
	return true;
}

