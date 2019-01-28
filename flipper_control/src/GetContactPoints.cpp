/*
 * FlipperControl.cpp
 *
 *  Created on: 12.12.2018
 *      Author: chfo
 */

#include "GetContactPoints.h"
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

GetContactPoints::GetContactPoints()
{

	tfListener = std::unique_ptr<tf::TransformListener> (new tf::TransformListener);

	// get namespace for tf
	tf_prefix = "//GETjag";
	tf_prefix = ros::this_node::getNamespace();

	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	BASE_FRAME = tf_prefix + "/base_link";
	ODOM_FRAME = tf_prefix + "/odom";
	MAP_FRAME = tf_prefix + "/map";

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
}

GetContactPoints::~GetContactPoints()
{

}


visualization_msgs::Marker GetContactPoints::createMarker (const std::string& tfFrame, const std::string& ns,const int& id,const double& x,const double& y,const  double& r,const double& g,const double& b,const double& a)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = tfFrame;
	marker.header.stamp = ros::Time();
	marker.ns = ns;

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(0);

	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a; // Don't forget to set the alpha!

    marker.id = id;

	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	return marker;
}


visualization_msgs::MarkerArray GetContactPoints::creatMarkerArrayFlipperPoints(const std::vector<geometry_msgs::Pose>& pose, const int& sign, const std::string& name, const std::string& frame, float r, float g, float b)
{
	geometry_msgs::Pose displayPose;
	visualization_msgs::MarkerArray markerArray;


	for(std::size_t i=0; i<pose.size();i++)
	{
		displayPose.position.x = pose[i].position.x + sign*deltaPose.position.x;
		displayPose.position.y = pose[i].position.y + sign*deltaPose.position.y;

		//displayPose = tfTransform(displayPose, MAP_FRAME, tf_prefix + frame);

		markerArray.markers.push_back (createMarker(tf_prefix + frame, tf_prefix + name, i, displayPose.position.x, displayPose.position.y, 1.0, 1.0, 0.0, 1.0));
	}
	return markerArray;
}

geometry_msgs::Pose GetContactPoints::clcDeltaPose(const geometry_msgs::Twist& velocitiy_robot, const double& delta_t)
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
}
cv::Mat GetContactPoints::getRobotRegions(cv::Mat mapImage, const geometry_msgs::Twist& velocitiy_robot, const double& delta_t)
{
	geometry_msgs::Pose pose;

	pose = clcDeltaPose(velocitiy_robot, delta_t);

	geometry_msgs::Pose flipperPose = tfTransform(pose, MAP_FRAME,BASE_FRAME);

	cv::Mat flipperMap;

	flipperMap = getCropedImage(flipperPose, mapImage, 2*TracksBaseLinkDist, FlipperTrackLength);

	return flipperMap;
}

std::vector<cv::Mat> GetContactPoints::getFlipperRegions(cv::Mat mapImage, const geometry_msgs::Twist& velocitiy_robot, const double& delta_t)
{
	geometry_msgs::Pose pose;

	pose = clcDeltaPose(velocitiy_robot, delta_t);

	pose.position.x += flipperLength/2;

	std::vector<cv::Mat> flipperMaps;
	cv::Mat flipperMap;

	geometry_msgs::Pose flipperPose = tfTransform(pose, MAP_FRAME, tf_prefix + "/flipper_front_left");

	flipperMap = getCropedImage(flipperPose, mapImage, yLength, flipperLength);
	flipperMaps.push_back(flipperMap);

	flipperPose = tfTransform(pose, MAP_FRAME, tf_prefix + "/flipper_front_right");
	flipperMap = getCropedImage(flipperPose, mapImage,  yLength, flipperLength);
	flipperMaps.push_back(flipperMap);

	flipperPose = tfTransform(pose ,MAP_FRAME, tf_prefix + "/flipper_rear_left");
	flipperMap = getCropedImage( flipperPose, mapImage,  yLength, flipperLength);
	flipperMaps.push_back(flipperMap);

	flipperPose = tfTransform(pose,MAP_FRAME, tf_prefix + "/flipper_rear_right");
	flipperMap = getCropedImage( flipperPose, mapImage, yLength, flipperLength);
	flipperMaps.push_back(flipperMap);

	return flipperMaps;
}

cv::Mat GetContactPoints::getCropedImage(geometry_msgs::Pose& pose, cv::Mat mapImage, double rectSizeX, double rectSizeY)
{
    mapSizeX = mapImage.cols;
    mapSizeY = mapImage.rows;

   	int x = mapSizeX/2 - pose.position.y/resultion;
   	int y = mapSizeY/2 - pose.position.x/resultion;


	int widthX = rectSizeX/resultion;
	//int widthY = FlipperTrackLength/resultion;

	int widthY = rectSizeY/resultion;


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

   /* cv::Mat copy = mapImage.clone();

    DrawRotatedRectangle(copy,rect );
    imshow("wadw", copy);
    waitKey(1);*/
	return cropped;
}


// Include center point of your rectangle, size of your rectangle and the degrees of rotation
void GetContactPoints::DrawRotatedRectangle(cv::Mat& image, cv::RotatedRect rotatedRectangle)
{
    cv::Scalar color = cv::Scalar(255.0, 255.0, 255.0); // white


    // We take the edges that OpenCV calculated for us
    cv::Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);

    // Convert them so we can use them in a fillConvexPoly
    cv::Point vertices[4];
    for(int i = 0; i < 4; ++i){
        vertices[i] = vertices2f[i];
    }

    // Now we can fill the rotated rectangle with our specified color
    cv::fillConvexPoly(image,
                       vertices,
                       4,
                       color);
}


std::vector<geometry_msgs::Pose> GetContactPoints::procGroundImage(cv::Mat flipperMaps)
{
	std::vector<geometry_msgs::Pose> poses;

	geometry_msgs::Pose pose;
	tf::Quaternion quat;
	quat.setRPY( 0, 0, 0);
	pose.orientation.x = quat.x();
	pose.orientation.y = quat.y();
	pose.orientation.z = quat.z();
	pose.orientation.w = quat.w();

	geometry_msgs::Pose flipperPose;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = 0;
	flipperPose.orientation.x = 0.0;
	flipperPose.orientation.y = 0.0;
	flipperPose.orientation.z = 0.0;
	flipperPose.orientation.w = 1.0;

	double cropLengthX = flipperMaps.rows*resultion;
	double cropLengthY = flipperMaps.cols*resultion;
	for(int i=0; i<flipperMaps.rows; i++)
	{
	    for(int j=0; j<flipperMaps.cols; j++)
	    {
	    	pose.position.x = cropLengthX/2 - cropLengthX/(flipperMaps.rows*2) - i*resultion;
	    	pose.position.y = cropLengthY/2 - cropLengthY/(flipperMaps.cols*2) - j*resultion;

	    	pose.position.z = flipperMaps.at<cv::Vec2b>(i, j)[0]*0.0043;

			flipperPose = pose;
			flipperPose = tfTransform(flipperPose, BASE_FRAME, MAP_FRAME);
			pose.position.z = flipperPose.position.z;
	    	poses.push_back(pose);
	    }
	}
	return poses;
}

std::vector<geometry_msgs::Pose> GetContactPoints::procFlipperMaps(cv::Mat flipperMaps, std::string flipperFrame)
{
	std::vector<geometry_msgs::Pose> poses;

	geometry_msgs::Pose pose;
	tf::Quaternion quat;
	quat.setRPY( 0, 0, 0);
	pose.orientation.x = quat.x();
	pose.orientation.y = quat.y();
	pose.orientation.z = quat.z();
	pose.orientation.w = quat.w();

	geometry_msgs::Pose flipperPose;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = 0;
	flipperPose.orientation.x = 0.0;
	flipperPose.orientation.y = 0.0;
	flipperPose.orientation.z = 0.0;
	flipperPose.orientation.w = 1.0;
	ROS_INFO("%s", flipperFrame.c_str());

	double cropLengthX = flipperMaps.rows*resultion;
	//double cropLengthY = flipperMaps.cols*resultion;
	for(int i=0; i<flipperMaps.rows; i++)
	{
	    for(int j=0; j<flipperMaps.cols; j++)
	    {
	    	pose.position.x = cropLengthX - cropLengthX/(flipperMaps.rows*2) - i * resultion ;
	    	pose.position.y = j*resultion;

	    	pose.position.z = flipperMaps.at<cv::Vec2b>(i, j)[0]*0.0043;
			ROS_INFO("pose before: x =%7.3lf, y=%7.3lf, z=%7.3lf", pose.position.x, pose.position.y, pose.position.z);

			flipperPose.position.z = pose.position.z;
			flipperPose = tfTransform(flipperPose, tf_prefix + flipperFrame, MAP_FRAME);
			pose.position.z = flipperPose.position.z;
	    	poses.push_back(pose);
			ROS_INFO("pose after: x =%7.3lf, y=%7.3lf, z=%7.3lf", pose.position.x, pose.position.y, pose.position.z);

	    }
	}
	return poses;
}

std::vector<geometry_msgs::Pose> GetContactPoints::transformPose(const std::vector<geometry_msgs::Pose>& poses,const std::string& destination_frame,const std::string& original_frame)
{
	std::vector<geometry_msgs::Pose> transformedPoses;
	geometry_msgs::Pose transPose;
	for(auto pose: poses)
	{
		transPose = tfTransform(pose, destination_frame, original_frame);
		transformedPoses.push_back(transPose);
	}
	return transformedPoses;
}

geometry_msgs::Pose GetContactPoints::tfTransform(const geometry_msgs::Pose& pose,const std::string& destination_frame,const std::string& original_frame)
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


