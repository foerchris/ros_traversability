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

	frontFlipperAngleDesiredPub = nodeHandle_.advertise < std_msgs::Float64 > ("flipper_front_controller/command", 1);
	rearFlipperAngleDesiredPub 	= nodeHandle_.advertise < std_msgs::Float64 > ("flipper_rear_controller/command", 1);

	planeRotationPub = nodeHandle_.advertise < sensor_msgs::Imu > ("fitted_plane_quaternion", 1);


	mapImageSet = false;

	// get namespace for tf and init all frames plus tf_prefix
	tf_prefix = "//GETjag";
	tf_prefix = ros::this_node::getNamespace();

	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	BASE_FRAME = "base_link";
	MAP_FRAME = "map";
	ODOM_FRAME = "odom";

	ROTATED_NEXT_BASE_FRAME = "rotated_next_base_link";

	FLIPPER_FRONT_LEFT_FRAME = "flipper_front_left";
	FLIPPER_FRONT_RIGHT_FRAME = "flipper_front_right";
	FLIPPER_REAR_LEFT_FRAME = "flipper_rear_left";
	FLIPPER_REAR_RIGHT_FRAME = "flipper_rear_right";

	FLIPPER_REGION_FRONT_LEFT_FRAME = "flipper_region_front_left";
	FLIPPER_REGION_FRONT_RIGHT_FRAME = "flipper_region_front_right";
	FLIPPER_REGION_REAR_LEFT_FRAME = "flipper_region_rear_left";
	FLIPPER_REGION_REAR_RIGHT_FRAME = "flipper_region_rear_right";


	NEXT_BASE_FRAME = tf_prefix + "/next_" + BASE_FRAME;
	BASE_FRAME = tf_prefix + "/" +BASE_FRAME;
	MAP_FRAME = tf_prefix + "/" + MAP_FRAME;
	ODOM_FRAME = tf_prefix + "/" + ODOM_FRAME;

	ROTATED_NEXT_BASE_FRAME = tf_prefix + "/" + ROTATED_NEXT_BASE_FRAME;

	FLIPPER_FRONT_LEFT_FRAME = tf_prefix + "/" + FLIPPER_FRONT_LEFT_FRAME;
	FLIPPER_FRONT_RIGHT_FRAME = tf_prefix + "/" + FLIPPER_FRONT_RIGHT_FRAME;
	FLIPPER_REAR_LEFT_FRAME = tf_prefix + "/" + FLIPPER_REAR_LEFT_FRAME;
	FLIPPER_REAR_RIGHT_FRAME = tf_prefix + "/" + FLIPPER_REAR_RIGHT_FRAME;

	FLIPPER_REGION_FRONT_LEFT_FRAME = tf_prefix + "/" + FLIPPER_REGION_FRONT_LEFT_FRAME;
	FLIPPER_REGION_FRONT_RIGHT_FRAME = tf_prefix + "/" + FLIPPER_REGION_FRONT_RIGHT_FRAME;
	FLIPPER_REGION_REAR_LEFT_FRAME = tf_prefix + "/" + FLIPPER_REGION_REAR_LEFT_FRAME;
	FLIPPER_REGION_REAR_RIGHT_FRAME = tf_prefix + "/" + FLIPPER_REGION_REAR_RIGHT_FRAME;

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
	double maxZ;
	tf2::Quaternion quat = groundPlane(mapImage, &maxZ);

	maxflipperContactPointsAngles angleFrontLeft = flipperRegion(mapImage, quat, maxZ, FLIPPER_FRONT_LEFT_FRAME,FLIPPER_REGION_FRONT_LEFT_FRAME);

	maxflipperContactPointsAngles angleFrontRight = flipperRegion(mapImage, quat, maxZ, FLIPPER_FRONT_RIGHT_FRAME, FLIPPER_REGION_FRONT_RIGHT_FRAME);

	maxflipperContactPointsAngles angleRearLeft = flipperRegion(mapImage, quat, maxZ, FLIPPER_REAR_LEFT_FRAME, FLIPPER_REGION_REAR_LEFT_FRAME);

	maxflipperContactPointsAngles angleRearRight = flipperRegion(mapImage, quat, maxZ, FLIPPER_REAR_RIGHT_FRAME, FLIPPER_REGION_REAR_RIGHT_FRAME);

	stabilityAnalysis(angleFrontLeft, angleFrontRight,FLIPPER_FRONT_LEFT_FRAME,FLIPPER_REAR_LEFT_FRAME);

	flipperAngles robotflipperAngles;
	robotflipperAngles.flipperAngleFront = returnBiggerVel(angleFrontLeft.maxFlipperAngle, angleFrontRight.maxFlipperAngle);
	robotflipperAngles.flipperAngleRear = returnBiggerVel(angleRearLeft.maxFlipperAngle, angleRearRight.maxFlipperAngle);

	/*flipperAngles robotflipperAngles;
	robotflipperAngles.flipperAngleFront = angleFrontLeft;
	robotflipperAngles.flipperAngleRear = 1;*/
	publishAngles(robotflipperAngles);

}



tf2::Quaternion FlipperControl::groundPlane(cv::Mat image, double* maxZValue)
{
	std::vector<geometry_msgs::Pose> groundContactPoints;

	groundContactPoints = getContactPoints.getRegions(image,FlipperTrackLength, 2*TracksBaseLinkDist, MAP_FRAME, NEXT_BASE_FRAME);

	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(groundContactPoints,"robot_disred_ground_pose", NEXT_BASE_FRAME,  1.0, 1.0, 0.0));
	FittedPlane fittedPlane = fitPlane.fitPlane(groundContactPoints);


	tf2::Quaternion quat = fitPlane.getRotations(fittedPlane);
	geometry_msgs::Pose pose;
	pose.position.x = 0;
	pose.position.y = 0;
	pose.position.z = 0;

	pose.orientation.x = quat.getX();
	pose.orientation.y = -quat.getY();
	pose.orientation.z = quat.getZ();
	pose.orientation.w = quat.getW();

	markerPublisher.publish(getContactPoints.creatCubeMarkerArrayFlipperPoints(pose,"fitted_plane",NEXT_BASE_FRAME,1.0, 0.0, 0.0));


	*maxZValue = getContactPoints.clcMaxZ(groundContactPoints,quat, trackLength-0.1);
	pose.position.z = *maxZValue;

	markerPublisher.publish(getContactPoints.creatCubeMarkerArrayFlipperPoints(pose,"fitted_shifted_plane",NEXT_BASE_FRAME,0.0, 1.0, 0.0));

	sensor_msgs::Imu pubRotation;
	pubRotation.orientation.x = quat.getX();
	pubRotation.orientation.y = -quat.getY();
	pubRotation.orientation.z = quat.getZ();
	pubRotation.orientation.w = quat.getW();

	planeRotationPub.publish(pubRotation);

	return quat;
}

maxflipperContactPointsAngles FlipperControl::flipperRegion(cv::Mat image,const tf2::Quaternion& quat, const double& maxZ, const std::string& flipperFrame, const std::string& flipperRegionFrame)
{

	std::vector<geometry_msgs::Pose> groundContactPoints;
	maxflipperContactPointsAngles flipperAngle;
	groundContactPoints = getContactPoints.getRegions(image,flipperLength, flipperWidth, MAP_FRAME, flipperRegionFrame);


	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(groundContactPoints,flipperRegionFrame, flipperRegionFrame,  1.0, 1.0, 0.0));


	std::vector<geometry_msgs::Pose> newGroundContactPoints;

	newGroundContactPoints = getContactPoints.clcNewFlipperPoses(groundContactPoints,quat, maxZ, NEXT_BASE_FRAME, flipperFrame, flipperRegionFrame);


	//***** display new points relativ to base_link should be the same as befor + offset
	newGroundContactPoints = getContactPoints.transformPose(newGroundContactPoints, NEXT_BASE_FRAME,  flipperFrame);

	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(newGroundContactPoints,flipperFrame +"_next", ROTATED_NEXT_BASE_FRAME,  1.0, 1.0, 0.0));

	newGroundContactPoints = getContactPoints.transformPose(newGroundContactPoints, flipperFrame, NEXT_BASE_FRAME);
	//*****

	flipperAngle = calcFlipperAngles.clcContactAngles(newGroundContactPoints);


	return flipperAngle;
}

void FlipperControl::stabilityAnalysis(const maxflipperContactPointsAngles& front,const maxflipperContactPointsAngles& rear,const std::string& frameFront, const std::string& frameRear)
{
	std::vector<geometry_msgs::Pose> pubvels;
	geometry_msgs::Pose pubvel;

	geometry_msgs::Pose centerOfGravity;
	centerOfGravity.position.x = 0;
	centerOfGravity.position.y = 0.275;
	centerOfGravity.position.z = 0.082;
	centerOfGravity.orientation.x = 0;
	centerOfGravity.orientation.y = 0;
	centerOfGravity.orientation.z = 0;
	centerOfGravity.orientation.w = 1;

	geometry_msgs::Pose rearTransfromed = getContactPoints.tfTransform(rear.pose, frameFront, frameRear);

	geometry_msgs::Pose centerOfGravityTransfromed = getContactPoints.tfTransform(centerOfGravity, frameFront, NEXT_BASE_FRAME);

	double S_NE = clcNESM.clcNESMStabilityMeasure(front.pose, rearTransfromed, centerOfGravityTransfromed);

	ROS_INFO("S_NE %3.4lf",S_NE);
	pubvels.clear();
	pubvel.position.x = clcNESM.g1_public[0];
	pubvel.position.y = clcNESM.g1_public[1];
	pubvel.position.z = clcNESM.g1_public[2];

	pubvels.push_back(pubvel);
	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(pubvels ,"g1", frameFront,  1.0, 0.0, 1.0));


	pubvels.clear();
	pubvel.position.x = clcNESM.g2_public[0];
	pubvel.position.y = clcNESM.g2_public[1];
	pubvel.position.z = clcNESM.g2_public[2];
	pubvels.push_back(pubvel);
	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(pubvels ,"g2", frameFront,  1.0, 0.0, 1.0));

	pubvels.clear();
	pubvel.position.x = clcNESM.c_public[0];
	pubvel.position.y = clcNESM.c_public[1];
	pubvel.position.z = clcNESM.c_public[2];
	pubvels.push_back(pubvel);
	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(pubvels ,"c", frameFront,  1.0, 0.0, 1.0));

	pubvels.clear();
	pubvel.position.x = clcNESM.g1_prime_public[0];
	pubvel.position.y = clcNESM.g1_prime_public[1];
	pubvel.position.z = clcNESM.g1_prime_public[2];

	pubvels.push_back(pubvel);
	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(pubvels ,"g1_prime", frameFront,  0.0, 0.0, 1.0));


	pubvels.clear();
	pubvel.position.x = clcNESM.g2_prime_public[0];
	pubvel.position.y = clcNESM.g2_prime_public[1];
	pubvel.position.z = clcNESM.g2_prime_public[2];
	pubvels.push_back(pubvel);
	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(pubvels ,"g2__prime", frameFront,  0.0, 0.0, 1.0));

	pubvels.clear();
	pubvel.position.x = clcNESM.c_prime_public[0];
	pubvel.position.y = clcNESM.c_prime_public[1];
	pubvel.position.z = clcNESM.c_prime_public[2];
	pubvels.push_back(pubvel);
	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(pubvels ,"c_prime", frameFront,  0.0, 0.0, 1.0));


	pubvels.clear();
	pubvel.position.x = clcNESM.p_highest_public[0];
	pubvel.position.y = clcNESM.p_highest_public[1];
	pubvel.position.z = clcNESM.p_highest_public[2];
	pubvels.push_back(pubvel);
	markerPublisher.publish(getContactPoints.creatMarkerArrayFlipperPoints(pubvels ,"p_highest", frameFront,  0.0, 0.0, 1.0));
}

double FlipperControl::returnBiggerVel(const double& vel1, const double& vel2)
{
	if(vel1>vel2)
	{
		return vel1;
	}
	else
	{
		return vel2;
	}
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

geometry_msgs::Pose FlipperControl::subPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
	geometry_msgs::Pose pose = pose1;
	pose.position.x = pose1.position.x -pose2.position.x;
	pose.position.y = pose1.position.y -pose2.position.y;
	pose.position.z = pose1.position.z -pose2.position.z;
	return pose;
}

geometry_msgs::Pose FlipperControl::addPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
	geometry_msgs::Pose pose = pose1;
	pose.position.x = pose1.position.x + pose2.position.x;
	pose.position.y = pose1.position.y + pose2.position.y;
	pose.position.z = pose1.position.z + pose2.position.z;
	return pose;
}


