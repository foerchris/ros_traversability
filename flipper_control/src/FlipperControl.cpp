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
	L = 0.2325;
	//d = std::hypot (r, L);
	theta = atan (r / L);
	dThreshold = sqrt(pow(R,2) + pow(L,2) - pow(R-r,2));
	resultion = 0.06;

	flipperWidth = 	 0.1;
	flipperLength =  L+r;
	trackLength = 0.6;
	FlipperTrackLength = 2*(flipperLength - R) + trackLength;
	TracksBaseLinkDist = 0.275;
	cropeMapLength = 2;

	getContactPoints.setConstants(resultion);
	calcFlipperAngles.setParameter(dThreshold, R, r, L);
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
	tf2::Quaternion quat = groundPlane(mapImage);

	flipperRegion(mapImage, quat, FLIPPER_FRONT_LEFT_FRAME);

	flipperRegion(mapImage, quat, FLIPPER_FRONT_RIGHT_FRAME);

	flipperRegion(mapImage, quat, FLIPPER_REAR_LEFT_FRAME);

	flipperRegion(mapImage, quat, FLIPPER_REAR_RIGHT_FRAME);

}



tf2::Quaternion FlipperControl::groundPlane(cv::Mat image)
{
	std::vector<geometry_msgs::Pose> groundContactPoints;

	groundContactPoints = getContactPoints.getRegions(image,FlipperTrackLength, 2*TracksBaseLinkDist, MAP_FRAME, NEXT_BASE_FRAME);

	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(groundContactPoints,"robot_disred_ground_pose", NEXT_BASE_FRAME,  1.0, 1.0, 0.0));
	FittedPlane fittedPlane = fitPlane.fitPlane(groundContactPoints);

	std::vector<geometry_msgs::Pose> planePoints = fitPlane.samplePlane(fittedPlane, FlipperTrackLength, 2*TracksBaseLinkDist, resultion);
	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(planePoints,"fitted plane points", NEXT_BASE_FRAME,  1.0, 1.0, 0.0));

	tf2::Quaternion quat = fitPlane.getRotations(fittedPlane);

	return quat;
}

double FlipperControl::flipperRegion(cv::Mat image,const tf2::Quaternion& quat, const std::string& flipperFrame)
{
	std::vector<geometry_msgs::Pose> groundContactPoints;
	double flipperAngle = 0;
	groundContactPoints = getContactPoints.getRegions(image,flipperLength, flipperWidth, MAP_FRAME, flipperFrame);

	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(groundContactPoints,flipperFrame, flipperFrame,  1.0, 1.0, 0.0));

	std::vector<geometry_msgs::Pose> newGroundContactPoints;

	newGroundContactPoints = getContactPoints.clcNewPoses(groundContactPoints,quat, NEXT_BASE_FRAME, flipperFrame);

	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(newGroundContactPoints,flipperFrame +"_next", flipperFrame,  1.0, 1.0, 0.0));

	flipperAngle = calcFlipperAngles.clcContactAngles(newGroundContactPoints);

	return flipperAngle;
}



