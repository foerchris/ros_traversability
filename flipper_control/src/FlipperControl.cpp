/*
 * FlipperControl.cpp
 *
 *  Created on: 12.12.2018
 *      Author: chfo
 */

#include <flipper_control/FlipperControl.h>

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


    // dynamic reconfigure
	static dynamic_reconfigure::Server<flipper_control::FlipperControlConfig> serverTrackCon (ros::NodeHandle ("~/FlipperControl"));
	static dynamic_reconfigure::Server<flipper_control::FlipperControlConfig>::CallbackType f2;
	f2 = boost::bind(&FlipperControl::reconfigureCallback,this,_1,_2);
	serverTrackCon.setCallback(f2);

	// get namespace for tf and init all frames plus tf_prefix
	tf_prefix = ros::this_node::getNamespace();
	if(tf_prefix == "/")
	{
		tf_prefix = "//GETjag1";
	}
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
	STATIC_NEXT_BASE_FRAME = tf_prefix + "/static" +"_next_" + BASE_FRAME;

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
	L = 0.23;
	//d = std::hypot (r, L);
	theta = atan (r / L);
	dThreshold = sqrt(pow(R,2) + pow(L,2) - pow(R-r,2));
	resultion = 0.06;

	flipperWidth = 	 0.1;
	flipperLength =  L+r;
	trackLength = 0.6;
	FlipperTrackLength = 2*(flipperLength - R) + trackLength;
	fitPlaneLength = FlipperTrackLength;
	TracksBaseLinkDist = 0.275;
	cropeMapLength = 2;

	cropMap.setConstants(resultion);
	flipperAngles.setParameter(dThreshold, R, r, L);

	S_NE_thresshold = 0.05;
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
		cv_ptr = cv_bridge::toCvCopy(msg, "32FC2");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat mapImage;
	cv_ptr->image.copyTo(mapImage);

	cv::Mat depth( mapImage.rows, mapImage.cols, CV_32FC1 );
	cv::Mat alpha( mapImage.rows, mapImage.cols, CV_32FC1 );

	// forming an array of matrices is a quite efficient operation,
	// because the matrix data is not copied, only the headers
	cv::Mat out[] = { depth,alpha };
	int from_to[] = { 0,0, 1,1};
	cv::mixChannels( &mapImage, 1, out, 2, from_to, 2 );

	depth.copyTo(globalMapImage);

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
	FittedPlane fittedPlane;
	double maxZ;

	tf2::Quaternion quat = groundPlane(mapImage, &maxZ, &fittedPlane);

	MaxFlipperContactPointsAngles angleFrontLeft = flipperRegion(mapImage, quat, maxZ, FLIPPER_FRONT_LEFT_FRAME,FLIPPER_REGION_FRONT_LEFT_FRAME);

	MaxFlipperContactPointsAngles angleFrontRight = flipperRegion(mapImage, quat, maxZ, FLIPPER_FRONT_RIGHT_FRAME, FLIPPER_REGION_FRONT_RIGHT_FRAME);

	MaxFlipperContactPointsAngles angleRearLeft = flipperRegion(mapImage, quat, maxZ, FLIPPER_REAR_LEFT_FRAME, FLIPPER_REGION_REAR_LEFT_FRAME);

	MaxFlipperContactPointsAngles angleRearRight = flipperRegion(mapImage, quat, maxZ, FLIPPER_REAR_RIGHT_FRAME, FLIPPER_REGION_REAR_RIGHT_FRAME);


	int nesmCase = cheakNESM(angleFrontLeft, angleFrontRight, angleRearLeft, angleRearRight);


	if(nesmCase == 2)
	{
		geometry_msgs::Pose pose;
		pose.position.x = 0;
		pose.position.y = 0;
		pose.position.z = 0;

		pose.orientation.x = 0;
		pose.orientation.y = 0;
		pose.orientation.z = 0;
		pose.orientation.w = 1.0;

		pose = cropMap.tfTransform(pose, NEXT_BASE_FRAME, STATIC_NEXT_BASE_FRAME);
		quat.setX(pose.orientation.x);
		quat.setY(pose.orientation.y);
		quat.setZ(pose.orientation.z);
		quat.setW(pose.orientation.w);

		angleFrontLeft = flipperRegion(mapImage, quat, maxZ, FLIPPER_FRONT_LEFT_FRAME,FLIPPER_REGION_FRONT_LEFT_FRAME);

		angleFrontRight = flipperRegion(mapImage, quat, maxZ, FLIPPER_FRONT_RIGHT_FRAME, FLIPPER_REGION_FRONT_RIGHT_FRAME);

		angleRearLeft = flipperRegion(mapImage, quat, maxZ, FLIPPER_REAR_LEFT_FRAME, FLIPPER_REGION_REAR_LEFT_FRAME);

		angleRearRight = flipperRegion(mapImage, quat, maxZ, FLIPPER_REAR_RIGHT_FRAME, FLIPPER_REGION_REAR_RIGHT_FRAME);
	}

	RobotFlipperAngles robotFlipperAngles;
	robotFlipperAngles.flipperAngleFront = returnBiggerVel(angleFrontLeft.maxFlipperAngle, angleFrontRight.maxFlipperAngle);
	robotFlipperAngles.flipperAngleRear = returnBiggerVel(angleRearLeft.maxFlipperAngle, angleRearRight.maxFlipperAngle);

	publishAngles(robotFlipperAngles);

}



tf2::Quaternion FlipperControl::groundPlane(cv::Mat image, double* maxZValue, FittedPlane* fittedPlane)
{
	std::vector<geometry_msgs::Pose> groundContactPoints;

	groundContactPoints = cropMap.getRegions(image,fitPlaneLength, 2*TracksBaseLinkDist, MAP_FRAME, NEXT_BASE_FRAME);

	markerPublisher.publish(cropMap.creatMarkerArrayFlipperPoints(groundContactPoints,"robot_disred_ground_pose", NEXT_BASE_FRAME,  1.0, 1.0, 0.0));
	*fittedPlane = fitPlane.fitPlane(groundContactPoints);


	tf2::Quaternion quat = fitPlane.getRotations(*fittedPlane);
	geometry_msgs::Pose pose;
	pose.position.x = 0;
	pose.position.y = 0;
	pose.position.z = 0;

	pose.orientation.x = quat.getX();
	pose.orientation.y = -quat.getY();
	pose.orientation.z = quat.getZ();
	pose.orientation.w = quat.getW();

	markerPublisher.publish(cropMap.creatCubeMarkerArrayFlipperPoints(pose,"fitted_plane",NEXT_BASE_FRAME,1.0, 0.0, 0.0));


	*maxZValue = cropMap.clcMaxZ(groundContactPoints,quat, trackLength-0.1);
	pose.position.z = *maxZValue;

	markerPublisher.publish(cropMap.creatCubeMarkerArrayFlipperPoints(pose,"fitted_shifted_plane",NEXT_BASE_FRAME,0.0, 1.0, 0.0));

	sensor_msgs::Imu pubRotation;
	pubRotation.orientation.x = quat.getX();
	pubRotation.orientation.y = -quat.getY();
	pubRotation.orientation.z = quat.getZ();
	pubRotation.orientation.w = quat.getW();

	planeRotationPub.publish(pubRotation);

	return quat;
}

tf2::Quaternion FlipperControl::newPlane(const cv::Mat& image,double* maxZValue, FittedPlane* fittedPlane, const double& setRoll, const double& setPitch)
{
	std::vector<geometry_msgs::Pose> groundContactPoints;

	groundContactPoints = cropMap.getRegions(image,FlipperTrackLength, 2*TracksBaseLinkDist, MAP_FRAME, NEXT_BASE_FRAME);

	markerPublisher.publish(cropMap.creatMarkerArrayFlipperPoints(groundContactPoints,"robot_disred_ground_pose", NEXT_BASE_FRAME,  1.0, 1.0, 0.0));

	tf2::Quaternion quat = fitPlane.getRotations(*fittedPlane);

	quat = cropMap.getDestQuat(quat, MAP_FRAME, NEXT_BASE_FRAME, setRoll, setPitch);

	geometry_msgs::Pose pose;
	pose.position.x = 0;
	pose.position.y = 0;
	pose.position.z = 0;

	pose.orientation.x = quat.getX();
	pose.orientation.y = -quat.getY();
	pose.orientation.z = quat.getZ();
	pose.orientation.w = quat.getW();

	markerPublisher.publish(cropMap.creatCubeMarkerArrayFlipperPoints(pose,"new_fitted_plane",NEXT_BASE_FRAME,1.0, 0.0, 0.0));


	*maxZValue = cropMap.clcMaxZ(groundContactPoints,quat, trackLength-0.1);
	pose.position.z = *maxZValue;

	markerPublisher.publish(cropMap.creatCubeMarkerArrayFlipperPoints(pose,"new_fitted_shifted_plane",NEXT_BASE_FRAME,0.0, 1.0, 0.0));

	sensor_msgs::Imu pubRotation;
	pubRotation.orientation.x = quat.getX();
	pubRotation.orientation.y = -quat.getY();
	pubRotation.orientation.z = quat.getZ();
	pubRotation.orientation.w = quat.getW();

	planeRotationPub.publish(pubRotation);

	return quat;
}

MaxFlipperContactPointsAngles FlipperControl::flipperRegion(cv::Mat image,const tf2::Quaternion& quat, const double& maxZ, const std::string& flipperFrame, const std::string& flipperRegionFrame)
{

	std::vector<geometry_msgs::Pose> groundContactPoints;
	MaxFlipperContactPointsAngles maxFlipperContactPointsAngles;
	groundContactPoints = cropMap.getRegions(image,flipperLength, flipperWidth, MAP_FRAME, flipperRegionFrame);


	markerPublisher.publish(cropMap.creatMarkerArrayFlipperPoints(groundContactPoints,flipperRegionFrame, flipperRegionFrame,  1.0, 1.0, 0.0));

	for(auto groundContactPoint:groundContactPoints)
	{
		//ROS_INFO("pose: x = %3.6lf, y = %3.6lf, z = %3.6lf", groundContactPoint.position.x, groundContactPoint.position.y, groundContactPoint.position.z);
	}

	std::vector<geometry_msgs::Pose> newGroundContactPoints;

	newGroundContactPoints = cropMap.clcNewFlipperPoses(groundContactPoints,quat, maxZ, NEXT_BASE_FRAME, flipperFrame, flipperRegionFrame);


	//***** display new points relativ to base_link should be the same as befor + offset
	newGroundContactPoints = cropMap.transformPose(newGroundContactPoints, NEXT_BASE_FRAME,  flipperFrame);

	markerPublisher.publish(cropMap.creatMarkerArrayFlipperPoints(newGroundContactPoints,flipperFrame +"_next", ROTATED_NEXT_BASE_FRAME,  1.0, 1.0, 0.0));

	newGroundContactPoints = cropMap.transformPose(newGroundContactPoints, flipperFrame, NEXT_BASE_FRAME);
	//*****

	maxFlipperContactPointsAngles = flipperAngles.clcContactAngles(newGroundContactPoints);

	std::vector<geometry_msgs::Pose> linePoints = fitPlane.sampleLine(maxFlipperContactPointsAngles.maxFlipperAngle, sqrt(pow(maxFlipperContactPointsAngles.pose.position.x,2) + pow(maxFlipperContactPointsAngles.pose.position.y,2)), 0.06);
	markerPublisher.publish(cropMap.creatMarkerArrayFlipperLine(linePoints,flipperFrame +"_line", flipperFrame,  0.0, 1.0, 0.0));

	return maxFlipperContactPointsAngles;
}

double FlipperControl::stabilityAnalysis(const geometry_msgs::Pose& g1,const geometry_msgs::Pose& g2, const geometry_msgs::Pose& c, const std::string& frame1, const std::string& frame2,  const bool& rotatePitch, const int& rotationDirection)
{
	double nESMValue = 0;
	geometry_msgs::Pose frontTransfromed = cropMap.tfTransform(g1, frame1, STATIC_NEXT_BASE_FRAME);

	geometry_msgs::Pose rearTransfromed = cropMap.tfTransform(g2, frame2, STATIC_NEXT_BASE_FRAME);

	geometry_msgs::Pose centerOfGravityTransfromed = cropMap.tfTransform(c, NEXT_BASE_FRAME, STATIC_NEXT_BASE_FRAME);

	nESMValue = nesm.clcNESMStabilityMeasure(frontTransfromed, rearTransfromed, centerOfGravityTransfromed, rotatePitch, rotationDirection);
	return nESMValue;
}

int FlipperControl::cheakNESM(MaxFlipperContactPointsAngles frontLeft, MaxFlipperContactPointsAngles frontRight, MaxFlipperContactPointsAngles rearLeft, MaxFlipperContactPointsAngles rearRight)
{
	geometry_msgs::Pose centerOfGravity;
	centerOfGravity.position.x = 0;
	centerOfGravity.position.y = 0;
	centerOfGravity.position.z = 0;
	centerOfGravity.orientation.x = 0;
	centerOfGravity.orientation.y = 0;
	centerOfGravity.orientation.z = 0;
	centerOfGravity.orientation.w = 1;

	double front_rear = stabilityAnalysis(frontLeft.pose, rearLeft.pose, centerOfGravity, FLIPPER_FRONT_LEFT_FRAME, FLIPPER_REAR_LEFT_FRAME, true, 1);
	//double left_right = stabilityAnalysis(frontLeft.pose, rearLeft.pose, centerOfGravity, FLIPPER_FRONT_LEFT_FRAME, FLIPPER_REAR_LEFT_FRAME, true, 1);

	//if(front_rear <= thresholdNESM && left_right <= thresholdNESM)
	//{
	//	return 3;
	//}
	if(front_rear <= S_NE_thresshold)
	{
		return 2;
	}
	//else if(left_right <= S_NE_thresshold)
	//{
	//	return 1;
	//}
	else
	{
		return 0;
	}
}

void FlipperControl::displayAllRotatedPoints(const std::string& position, const std::string& frame)
{
	displayRotatedPoints(nesm.g1_public, position + "g1", frame, 0.0, 0.0, 1.0);
	displayRotatedPoints(nesm.g2_public, position + "g2", frame, 0.0, 0.0, 1.0);
	displayRotatedPoints(nesm.c_public, position + "c", frame, 0.0, 0.0, 1.0);
	displayRotatedPoints(nesm.g1_prime_public, position + "g1_prime", frame, 1.0, 0.0, 0.0);
	displayRotatedPoints(nesm.g2_prime_public, position + "g2_prime", frame, 1.0, 0.0, 0.0);
	displayRotatedPoints(nesm.c_prime_public, position + "c_prime", frame, 1.0, 0.0, 0.0);
	displayRotatedPoints(nesm.p_highest_public, position + "p_highest_public", frame, 0.0, 1.0, 0.0);

}
void FlipperControl::displayRotatedPoints(const cv::Vec3d& point, const std::string& name, const std::string& frame, float r, float g, float b)
{
	std::vector<geometry_msgs::Pose> poses;
	geometry_msgs::Pose pose;
	poses.clear();
	pose.position.x = point[0];
	pose.position.y = point[1];
	pose.position.z = point[2];
	poses.push_back(pose);
	markerPublisher.publish(cropMap.creatMarkerArrayFlipperPoints(poses , name, frame,  0.0, 0.0, 1.0));
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

void FlipperControl::publishAngles (RobotFlipperAngles robotFlipperAngles)
{
	static std_msgs::Float64 frontFlipperAngleDesiredMsg;
	static std_msgs::Float64 rearFlipperAngleDesiredMsg;
	if (robotFlipperAngles.flipperAngleFront != NAN)
	{
		frontFlipperAngleDesiredMsg.data = -robotFlipperAngles.flipperAngleFront;
	}
	else
	{
		//ROS_INFO (" Front angle NAN:\t  [%7.3f]", robotFlipperAngles.flipperAngleFront);
	}

	if (robotFlipperAngles.flipperAngleRear != NAN)
	{
		rearFlipperAngleDesiredMsg.data = -robotFlipperAngles.flipperAngleRear;
	}
	else
	{
		//ROS_INFO (" Rear angle NAN:\t  [%7.3f]", robotFlipperAngles.flipperAngleRear);
	}

	if(frontFlipperAngleDesiredMsg.data<=M_PI/2 && frontFlipperAngleDesiredMsg.data>=-M_PI/2)
	{
		//std::cout<<"frontFlipperAngleDesiredMsg.data"<<frontFlipperAngleDesiredMsg.data<<std::endl;
		frontFlipperAngleDesiredPub.publish (frontFlipperAngleDesiredMsg);
	}

	if(rearFlipperAngleDesiredMsg.data<=M_PI/2 && rearFlipperAngleDesiredMsg.data>=-M_PI/2)
	{
		//std::cout<<"rearFlipperAngleDesiredPub.data"<<rearFlipperAngleDesiredMsg.data<<std::endl;
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

void FlipperControl::reconfigureCallback(flipper_control::FlipperControlConfig&config, uint32_t level)
{
	R = config.R;
	r = config.r;
	L = config.L;
	trackLength = config.trackLength;
	TracksBaseLinkDist = config.TracksBaseLinkDist;
	cropeMapLength = config.cropeMapLength;
	fitPlaneLength = config.fitPlaneLength;
}


