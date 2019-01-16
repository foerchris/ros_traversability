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

FlipperControl::FlipperControl(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{

	static image_transport::ImageTransport it(nodeHandle_);
	static image_transport::Subscriber it_sub;
	it_sub = it.subscribe("elevation_map_image", 1, boost::bind (&FlipperControl::MapImageCallback, this, _1));
	tfListener = std::unique_ptr<tf::TransformListener> (new tf::TransformListener);

	frontFlipperAngleDesiredPub = nodeHandle_.advertise < std_msgs::Float64 > ("flipper_front_controller/command", 1);
	rearFlipperAngleDesiredPub 	= nodeHandle_.advertise < std_msgs::Float64 > ("flipper_rear_controller/command", 1);

	tf_prefix = "//GETjag";
	tf_prefix = ros::this_node::getNamespace();

	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	BASE_FRAME = tf_prefix + "/base_link";
	ODOM_FRAME = tf_prefix + "/odom";
	MAP_FRAME = tf_prefix + "/map";

	// Roboter paramter
	R = 0.082;
	r = 0.0375;
	L = 0.22;
	//d = std::hypot (r, L);
	theta = atan (r / L);
	dThreshold = sqrt(r*r + L*L + (R-r)*(R-r));
	resultion = 0.06;
	xLength  = 0.1;
	yLength  = 0.2575;
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
	SequenceControl(mapImage);

}

void FlipperControl::SequenceControl(cv::Mat mapImage)
{
	cv::Point min_loc, max_loc;

	std::vector<cv::Mat> flipperMaps;

	flipperMaps = getFlipperRegions(mapImage);

	//std::vector<minMaxFlipperVel> minMaxValues;

	//minMaxValues = clcMinMaxVels(flipperMaps);

	std::vector<geometry_msgs::Pose> contactPoints;

	contactPoints = getContactPoints(flipperMaps[0]);
	flipperContactPointsAngles flipperFrontLeft;
	flipperFrontLeft = clcContactAngles(contactPoints, "/static_flipper_front_left");

	contactPoints = getContactPoints(flipperMaps[1]);
	flipperContactPointsAngles flipperFrontRight;
	flipperFrontRight = clcContactAngles(contactPoints, "/static_flipper_front_right");

	contactPoints = getContactPoints(flipperMaps[2]);
	flipperContactPointsAngles flipperRearRight;
	flipperRearRight = clcContactAngles(contactPoints, "/static_flipper_rear_left");

	contactPoints = getContactPoints(flipperMaps[3]);
	flipperContactPointsAngles flipperRearLeft;
	flipperRearLeft = clcContactAngles(contactPoints, "/static_flipper_rear_right");

	//flipperAngles robotFlipperAngles;
	//clcFlipperAngles(minMaxValues, robotFlipperAngles);

	//ROS_INFO("robotFlipperAngles:  flipperAngleFront = %lf, flipperAngleRear = %lf\n", robotFlipperAngles.flipperAngleFront, robotFlipperAngles.flipperAngleRear);

	//publishAngles(robotFlipperAngles);
}

flipperContactPointsAngles FlipperControl::clcContactAngles(const std::vector<geometry_msgs::Pose>& values, std::string flipperFrame)
{
	flipperContactPointsAngles robotFlipperAngles;

	geometry_msgs::Pose flipperPose;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = 0;
	tf2::Quaternion quat;
	quat.setRPY(0,0, 0);
	flipperPose.orientation.x = quat.x();
	flipperPose.orientation.y = quat.y();
	flipperPose.orientation.z = quat.z();
	flipperPose.orientation.w = quat.w();

	double z = 0;
	double x = 0;
	double d = 0;
	for(std::size_t i = 0; i <values.size(); i++)
	{
		geometry_msgs::Pose point;

		double phi1;
		double phi2;
		double phiContact;
		double flipperAngle;

		point = values[i];


		z = values[i].position.z;
		x = values[i].position.x;

		flipperPose.position.z = z;
		flipperPose = mapToFlipperTransform(flipperPose, tf_prefix + flipperFrame);
		z = flipperPose.position.z;

		point.position.z=z;

		d= sqrt(pow(x, 2)+ pow(z, 2));


		if(d<= dThreshold)
		{
			phi1 = atan(z/x);
			phi2 = asin(R/sqrt(pow(x, 2) + pow(z,2)));
			phiContact = phi1 + phi2;

			//********************************************************** nochmal nachrechnen **********************************************************
			flipperAngle = phi1 - asin((R-r)/L);
		}
		else
		{
			phi1 = asin((pow(d,2) + pow(L,2) - pow(R,2))/ (2*L*d)) - atan(x/z);
			phi2 = asin((R-r)/L);
			phiContact = phi1 + phi2;

			//********************************************************** nochmal nachrechnen **********************************************************
			flipperAngle = phi1;
		}
		robotFlipperAngles.pose.push_back(point);
		robotFlipperAngles.phi1.push_back(phi1);
		robotFlipperAngles.phi2.push_back(phi2);
		robotFlipperAngles.phiContact.push_back(phiContact);
		robotFlipperAngles.flipperAngle.push_back(flipperAngle);

	}

	return robotFlipperAngles;
}

void FlipperControl::clcFlipperAngles(const std::vector<minMaxFlipperVel>& minMaxVel, flipperAngles& robotFlipperAngles)
{
	geometry_msgs::Pose flipperPose;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = 0;
	tf2::Quaternion quat;
	quat.setRPY(0,0, 0);
	flipperPose.orientation.x = quat.x();
	flipperPose.orientation.y = quat.y();
	flipperPose.orientation.z = quat.z();
	flipperPose.orientation.w = quat.w();


	double zFrontLeftMax = minMaxVel[0].max*0.0043;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = zFrontLeftMax;
	flipperPose = mapToFlipperTransform(flipperPose, tf_prefix + "/static_flipper_front_left");
	zFrontLeftMax = flipperPose.position.z;

	double zFrontRightMax = minMaxVel[1].max*0.0043;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = zFrontRightMax;
	flipperPose = mapToFlipperTransform(flipperPose, tf_prefix + "/static_flipper_front_right");
	zFrontRightMax = flipperPose.position.z;

	double zRearLeftMax = minMaxVel[2].max*0.0043;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = zRearLeftMax;
	flipperPose = mapToFlipperTransform(flipperPose, tf_prefix + "/static_flipper_rear_left");
	zRearLeftMax = flipperPose.position.z;

	double zRearRightMax = minMaxVel[3].max*0.0043;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = zRearRightMax;
	flipperPose = mapToFlipperTransform(flipperPose, tf_prefix + "/static_flipper_rear_right");
	zRearRightMax = flipperPose.position.z;

	double xFront = 0;
	double zFront = 0;
	double xRear  = 0;
	double zRear  = 0;

	if(zFrontRightMax > zFrontLeftMax)
	{
		xFront = yLength - minMaxVel[1].max_loc.y*resultion;
		zFront = zFrontRightMax;
	}
	else
	{
		xFront = yLength - minMaxVel[0].max_loc.y*resultion;
		zFront = zFrontLeftMax;
	}

	if(zRearRightMax > zRearLeftMax)
	{
		xRear = yLength - minMaxVel[3].max_loc.y*resultion;
		zRear = zRearRightMax;

	}
	else
	{
		xRear = yLength - minMaxVel[2].max_loc.y*resultion;
		zRear = zRearLeftMax;

	}
	ROS_INFO("xFront = %lf, zFront = %lf\n", xFront, zFront);
	ROS_INFO("xRear = %lf, zRear = %lf\n", xRear, zRear);

	double dFront;
	double dRear;

	dFront = sqrt( pow(zFront, 2) + pow(xFront, 2));
	dRear = sqrt( pow(zRear, 2) + pow(xRear, 2));
	//ROS_INFO("dFront = %lf, dRear = %lf\n", dFront, dRear);

	// to do **********************************************************************************
	/// nochmal genau die einzelnden zustände nachdenken gerade werden negative hindernisse nicht perfekt behandelt
	if(zFront!=0)
	{
		if(xFront!=0)
		{
			double phi1 = 0;
			double phi2 = 0;
			double phiContact = 0;
			if(dFront <= dThreshold)
			{
				phi1 = atan(zFront/xFront);
				phi2 = asin(R/sqrt(pow(xFront, 2) + pow(zFront,2)));
				phiContact = phi1 + phi2;
				robotFlipperAngles.flipperAngleFront = phi1;
			}
			else
			{
				phi1 = asin((pow(dFront,2) + pow(L,2) - pow(R,2))/ (2*L*dFront)) - atan(xFront/zFront);
				phi2 = asin((R-r)/L);
				phiContact = phi1 + phi2;

				robotFlipperAngles.flipperAngleFront = phi1;
			}
			ROS_INFO("phi1 = %lf, phi2 = %lf, phiContact = %lf\n", phi1, phi2, phiContact);
			ROS_INFO("robotFlipperAngles.flipperAngleFront = %lf\n", robotFlipperAngles.flipperAngleFront);

		}
		else
		{
			robotFlipperAngles.flipperAngleFront = 85;
		}
	}
	else
	{
		robotFlipperAngles.flipperAngleFront = 0;
	}

	if(zRear!=0)
	{
		if(xRear!=0)
		{
			double phi1 = 0;
			double phi2 = 0;
			double phiContact = 0;
			if(dRear <= dThreshold)
			{
				phi1 = atan(zRear/xRear);
				phi2 = asin(R/sqrt(pow(xRear, 2) + pow(zRear,2)));
				phiContact = phi1 + phi2;

				robotFlipperAngles.flipperAngleRear = phi1;
			}
			else
			{
				phi1 = asin((pow(dRear,2) + pow(L,2) - pow(R,2))/ (2*L*dRear)) - atan(xRear/zRear);
				phi2 = asin((R-r)/L);
				phiContact = phi1 + phi2;

				robotFlipperAngles.flipperAngleRear = asin((pow(dRear,2) + pow(L,2) - pow(R,2))/ (2*L*dRear)) - atan(xRear/zRear) + asin((R-r)/L);
			}
			ROS_INFO("phi1 = %lf, phi2 = %lf, phiContact = %lf\n", phi1, phi2, phiContact);
			ROS_INFO("robotFlipperAngles.flipperAngleFront = %lf\n", robotFlipperAngles.flipperAngleFront);
		}
		else
		{
			robotFlipperAngles.flipperAngleRear = 85;
		}
	}
	else
	{
		robotFlipperAngles.flipperAngleRear = 0;
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

std::vector<geometry_msgs::Pose> FlipperControl::getContactPoints(cv::Mat flipperMaps)
{
	std::vector<geometry_msgs::Pose> poses;

	geometry_msgs::Pose pose;
	tf::Quaternion quat;
	quat.setRPY( 0, 0, 0);
	pose.orientation.x = quat.x();
	pose.orientation.y = quat.y();
	pose.orientation.z = quat.z();
	pose.orientation.w = quat.w();

	for(int i=0; i<flipperMaps.rows; i++)
	{
	    for(int j=0; j<flipperMaps.cols; j++)
	    {
	    	pose.position.x = yLength - i*resultion;
	    	pose.position.y = yLength - j*resultion;
	    	cv::Point point(j,i);
	    	pose.position.z = flipperMaps.at<cv::Vec2i>(i, j)[0]*0.0043;
	    	poses.push_back(pose);
	    }
	}
	return poses;
}

std::vector<minMaxFlipperVel> FlipperControl::clcMinMaxVels(std::vector<cv::Mat> flipperMaps)
{
	std::vector<minMaxFlipperVel> values;
	minMaxFlipperVel value;

	for(std::size_t i=0; i<flipperMaps.size(); i++)
	{
		cv::minMaxLoc(flipperMaps[i], &value.min, &value.max, &value.min_loc, &value.max_loc);
		values.push_back(value);
	}
	return values;
}

std::vector<cv::Mat> FlipperControl::getFlipperRegions(cv::Mat mapImage)
{
	geometry_msgs::Pose pose;
	pose.position.x = 0.12875;
	pose.position.y = 0.0;
	pose.position.z = 0.0;
	tf::Quaternion quat;
	quat.setRPY( 0, 0, 0);

	pose.orientation.x = quat.x();
	pose.orientation.y = quat.y();
	pose.orientation.z = quat.z();
	pose.orientation.w = quat.w();

	std::vector<cv::Mat> flipperMaps;
	cv::Mat flipperMap;

	geometry_msgs::Pose flipperPose = flipperToMapTransform(pose, tf_prefix + "/static_flipper_front_left");

	flipperMap = getCropedImage(flipperPose, mapImage);
	flipperMaps.push_back(flipperMap);

	flipperPose = flipperToMapTransform(pose, tf_prefix + "/static_flipper_front_right");
	flipperMap = getCropedImage(flipperPose, mapImage);
	flipperMaps.push_back(flipperMap);

	flipperPose = flipperToMapTransform(pose, tf_prefix + "/static_flipper_rear_left");
	flipperMap = getCropedImage(flipperPose, mapImage);
	flipperMaps.push_back(flipperMap);

	flipperPose = flipperToMapTransform(pose, tf_prefix + "/static_flipper_rear_right");
	flipperMap = getCropedImage(flipperPose, mapImage);
	flipperMaps.push_back(flipperMap);

	return flipperMaps;

}

cv::Mat FlipperControl::getCropedImage(geometry_msgs::Pose& pose, cv::Mat mapImage)
{


    mapSizeX = mapImage.cols;
    mapSizeY = mapImage.rows;

	int x = mapSizeX/2 - pose.position.y/resultion;
	int y = mapSizeY/2 - pose.position.x/resultion;

	int widthX = xLength/resultion;
	int widthY =  yLength/resultion;

	cv::Point2f point(x,y);
	cv::Size2f size(widthX,widthY);
	tf::Quaternion quat(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);

	float angle = -tf::getYaw(quat)/M_PI*180;
    // rect is the RotatedRect

	cv::RotatedRect rect(point, size,  angle);
    // matrices we'll use
    Mat M, rotated, cropped;
    // get angle and size from the bounding box
    Size rect_size = rect.size;

    // get the rotation matrix
    M = getRotationMatrix2D(rect.center, angle, 1.0);
    // perform the affine transformation
    warpAffine(mapImage, rotated, M, mapImage.size(), 2);
    // crop the resulting image
    getRectSubPix(rotated, rect_size, rect.center, cropped);
	//imshow("input",mapImage);

	//imshow("output",cropped);
	std::cout<<"cropped"<<cropped<<std::endl;
	//waitKey(1);
	return cropped;
}

geometry_msgs::Pose FlipperControl::flipperToMapTransform(const geometry_msgs::Pose& pose,const std::string& tfID)
{
	// TF transformation of the Point which is nearest to the robot
	const ros::Time& scanTimeStamp = ros::Time (0);

    //std::cout<<"flipperToMapTransform"<<tfID<<std::endl;
	//ROS_INFO("pose:  x = %lf, y = %lf, z = %lf\n", pose.position.x, pose.position.y, pose.position.z);

    try
	{
		tfListener->waitForTransform (MAP_FRAME, tfID, scanTimeStamp, ros::Duration (3.0));
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

	tf::Stamped<tf::Pose> transPose (tf::Pose( quat,origin), scanTimeStamp, tfID);

	tf::Stamped<tf::Pose> poseTransformed;

	try
	{
		tfListener->transformPose(MAP_FRAME, scanTimeStamp, transPose, tfID, poseTransformed);
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
	//ROS_INFO("returnPose:  x = %lf, y = %lf, z = %lf\n", returnPose.position.x, returnPose.position.y, returnPose.position.z);

	return returnPose;
}

geometry_msgs::Pose FlipperControl::mapToFlipperTransform(const geometry_msgs::Pose& pose,const std::string& tfID)
{
	// TF transformation of the Point which is nearest to the robot
	const ros::Time& scanTimeStamp = ros::Time (0);

    //std::cout<<"mapToFlipperTransform"<<tfID<<std::endl;
	//ROS_INFO("pose:  x = %lf, y = %lf, z = %lf\n", pose.position.x, pose.position.y, pose.position.z);

    try
	{
		tfListener->waitForTransform (tfID, MAP_FRAME, scanTimeStamp, ros::Duration (3.0));
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

	tf::Stamped<tf::Pose> transPose (tf::Pose( quat,origin), scanTimeStamp, MAP_FRAME);

	tf::Stamped<tf::Pose> poseTransformed;

	try
	{
		tfListener->transformPose(tfID, scanTimeStamp, transPose, MAP_FRAME, poseTransformed);
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
	//ROS_INFO("returnPose:  x = %lf, y = %lf, z = %lf\n", returnPose.position.x, returnPose.position.y, returnPose.position.z);

	return returnPose;
}
