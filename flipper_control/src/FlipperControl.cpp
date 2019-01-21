/*
 * FlipperControl.cpp
 *
 *  Created on: 12.12.2018
 *      Author: chfo
 */

#include "FlipperControl.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>

#include <std_msgs/Float64.h>

using namespace cv;

constexpr double D2R = M_PI / 180.0;
constexpr double R2D = 180.0 / M_PI;

FlipperControl::FlipperControl(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{

	static image_transport::ImageTransport it(nodeHandle_);
	static image_transport::Subscriber it_sub;
	it_sub = it.subscribe("elevation_map_image", 1, boost::bind (&FlipperControl::MapImageCallback, this, _1));

	markerPublisher = nodeHandle_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 20);
    odomSub = nodeHandle_.subscribe<nav_msgs::Odometry> ("odom", 1, &FlipperControl::odomCallback,this);

	frontFlipperAngleDesiredPub = nodeHandle_.advertise < std_msgs::Float64 > ("flipper_front_controller/command", 1);
	rearFlipperAngleDesiredPub 	= nodeHandle_.advertise < std_msgs::Float64 > ("flipper_rear_controller/command", 1);

	msg_timer = nodeHandle_.createTimer(ros::Duration(0.8), boost::bind (&FlipperControl::FlipperSequenzCallback, this, _1));


	mapImageSet = false;

	// get namespace for tf
	tf_prefix = "//GETjag";
	tf_prefix = ros::this_node::getNamespace();

	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);
	start_time = ros::Time::now();
}

FlipperControl::~FlipperControl()
{
	nodeHandle_.shutdown();
}


void FlipperControl::MapImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, "8UC1");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat mapImage = cv_ptr->image;

	mapImage.copyTo(globalMapImage);
	//ROS_INFO (" MapImageCallback");

	mapImageSet = true;
}


void FlipperControl::FlipperSequenzCallback(const ros::TimerEvent& event)
{
	if(mapImageSet)
	{
		SequenceControl(globalMapImage);
	}
}

void FlipperControl::SequenceControl(cv::Mat mapImage)
{
	cv::Point min_loc, max_loc;

	tf2::Quaternion quat= groundPlane(mapImage);

	std::vector<cv::Mat> flipperImage;
	flipperImage = getContactPoints.getFlipperRegions(mapImage);


	delta_t = (ros::Time::now() - start_time).toSec();

	//*********************** calculations for the the left flippers
	flipperEval("/flipper_front_left",flipperImage[0], quat, 1);


	flipperEval("/flipper_front_right",flipperImage[1], quat, 1);

	//*********************** calculations for the the right flippers
	flipperEval("/flipper_rear_left",flipperImage[2], quat, -1);

	flipperEval("/flipper_rear_right",flipperImage[3], quat, -1);

	start_time = ros::Time::now();


	//publishAngles(robotFlipperAngles);
}


tf2::Quaternion FlipperControl::groundPlane(cv::Mat image)
{
	cv::Mat robotGroundImage;

	robotGroundImage = getContactPoints.getRobotRegions(image);

	std::vector<geometry_msgs::Pose> groundContactPoints;

	groundContactPoints = getContactPoints.procGroundImage(robotGroundImage);
	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(groundContactPoints, "/flipper_pose", "/base_link",  1.0, 1.0, 0.0));

	tf2::Quaternion quat;
	groundContactPoints = fitPlane.isInRobotRange(groundContactPoints, currentVelocity.linear.x, delta_t);
	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(groundContactPoints, "/new_flipper_pose", "/base_link",  1.0, 1.0, 0.0));

	quat = fitPlane.fitPlane(groundContactPoints);

	return quat;
}


void FlipperControl::flipperEval(const std::string& flipper, cv::Mat image,const tf2::Quaternion& quat, int frontRear)
{
	std::vector<geometry_msgs::Pose> flipperContactPoints;
	flipperContactPoints = getContactPoints.procFlipperMaps(image, flipper);
	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(flipperContactPoints, "/flipper_pose" + flipper, flipper,  1.0, 1.0, 0.0));

	// Funktion in range for the flipper positions
	flipperContactPoints = fitPlane.isInFlipperRange(flipperContactPoints, frontRear*currentVelocity.linear.x, delta_t);

	std::vector<geometry_msgs::Pose> newtracksContactPoints;
	newtracksContactPoints = calcFlipperAngles.clcNewPoses(flipperContactPoints,quat);
	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(flipperContactPoints, "/new_flipper_pose" + flipper, flipper,  1.0, 1.0, 0.0));

}


void FlipperControl::publishAngles (flipperAngles robotFlipperAngles)
{
	static std_msgs::Float64 frontFlipperAngleDesiredMsg;
	static std_msgs::Float64 rearFlipperAngleDesiredMsg;
	if (robotFlipperAngles.flipperAngleFront != NAN)
	{
		frontFlipperAngleDesiredMsg.data = -robotFlipperAngles.flipperAngleFront;
	}
	else
		ROS_INFO (" Front angle NAN:\t  [%7.3f]", robotFlipperAngles.flipperAngleFront);
	if (robotFlipperAngles.flipperAngleRear != NAN)
	{
		rearFlipperAngleDesiredMsg.data = -robotFlipperAngles.flipperAngleRear;
	}
	else
		ROS_INFO (" Rear angle NAN:\t  [%7.3f]", robotFlipperAngles.flipperAngleRear);


	if(frontFlipperAngleDesiredMsg.data<=M_PI/2 && frontFlipperAngleDesiredMsg.data>=-M_PI/2)
	{
		frontFlipperAngleDesiredPub.publish (frontFlipperAngleDesiredMsg);
	}

	if(rearFlipperAngleDesiredMsg.data<=M_PI/2 && rearFlipperAngleDesiredMsg.data>=-M_PI/2)
	{
		rearFlipperAngleDesiredPub.publish (rearFlipperAngleDesiredMsg);

	}
}

void FlipperControl::odomCallback (const nav_msgs::OdometryConstPtr& odomMsg)
{

	currentVelocity = odomMsg->twist.twist;
	//std::cout<<"currentVelocity: "<< currentVelocity.linear.x<<std::endl;
}



