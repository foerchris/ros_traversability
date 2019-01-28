/*
 * FlipperControl.cpp
 *
 *  Created on: 12.12.2018
 *      Author: chfo
 */

#include "FlipperControl.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>

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


	msg_timer = nodeHandle_.createTimer(ros::Duration(0.8), boost::bind (&FlipperControl::FlipperSequenzCallback, this, _1));


	mapImageSet = false;

	// get namespace for tf and init all frames plus tf_prefix
	tf_prefix = "//GETjag";
	tf_prefix = ros::this_node::getNamespace();

	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	BASE_FRAME = "base_link";
	MAP_FRAME = "map";
	ODOM_FRAME = "odom";
	FLIPPER_FRONT_LEFT_FRAME = "flipper_region_front_left";
	FLIPPER_FRONT_RIGHT_FRAME = "flipper_region_front_right";
	FLIPPER_REAR_LEFT_FRAME = "flipper_region_rear_left";
	FLIPPER_REAR_RIGHT_FRAME = "flipper_region_rear_right";

	NEXT_BASE_FRAME = tf_prefix + "/next_" + BASE_FRAME;
	BASE_FRAME = tf_prefix + "/" +BASE_FRAME;
	MAP_FRAME = tf_prefix + "/" + MAP_FRAME;
	ODOM_FRAME = tf_prefix + "/" + ODOM_FRAME;

	FLIPPER_FRONT_LEFT_FRAME = tf_prefix + "/" + FLIPPER_FRONT_LEFT_FRAME;
	FLIPPER_FRONT_RIGHT_FRAME = tf_prefix + "/" + FLIPPER_FRONT_RIGHT_FRAME;
	FLIPPER_REAR_LEFT_FRAME = tf_prefix + "/" + FLIPPER_REAR_LEFT_FRAME;
	FLIPPER_REAR_RIGHT_FRAME = tf_prefix + "/" + FLIPPER_REAR_RIGHT_FRAME;

	// init mapsize
	mapSizeX = 200;
	mapSizeY = 200;

	// Roboter paramter
	R = 0.082;
	r = 0.0375;
	//L = 0.22;
	L = 0.2325;
	//d = std::hypot (r, L);
	theta = atan (r / L);
	dThreshold = sqrt(pow(R,2) + pow(L,2) - pow(R-r,2));
	resultion = 0.06;
	//xLength  = 0.1;
	yLength  = 0.1;
	flipperWidth = yLength;
	//yLength  = 0.2575;
	//	yLength = L+r;
	xLength = L+r;
	flipperLength = xLength;
	trackLength = 0.6;
	FlipperTrackLength = 2*(xLength - R) + trackLength;
	TracksBaseLinkDist = 0.275;
	cropeMapLength = 2;

	getContactPoints.setConstants(resultion);
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
	tf2::Quaternion quat= groundPlane(mapImage);
}



tf2::Quaternion FlipperControl::groundPlane(cv::Mat image)
{
	cv::Mat robotGroundImage;

	robotGroundImage = getContactPoints.getRegions(image,FlipperTrackLength, 2*TracksBaseLinkDist, MAP_FRAME, NEXT_BASE_FRAME);

	ROS_INFO("robotGroundImage: cols = %i , rows = %i",robotGroundImage.cols, robotGroundImage.rows);
	std::vector<geometry_msgs::Pose> groundContactPoints;

	//groundContactPoints = getContactPoints.procGroundImage(robotGroundImage, BASE_FRAME, MAP_FRAME);

	//markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(groundContactPoints, 1,"/flipper_pose", "/base_link",  1.0, 1.0, 0.0));

	tf2::Quaternion quat;


	return quat;
}




