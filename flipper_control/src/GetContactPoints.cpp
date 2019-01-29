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
	mapSizeX = 200;
	mapSizeY = 200;
	resultion = 0.06;
}

GetContactPoints::~GetContactPoints()
{

}
void GetContactPoints::setConstants(const double& imageResultion)
{
	resultion = imageResultion;
}


visualization_msgs::Marker GetContactPoints::createMarker (const std::string& tfFrame, const std::string& ns,const int& id,const double& x,const double& y, const double& z,const  double& r,const double& g,const double& b,const double& a)
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
	marker.pose.position.z = z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	return marker;
}


visualization_msgs::MarkerArray GetContactPoints::creatMarkerArrayFlipperPoints(const std::vector<geometry_msgs::Pose>& pose, const std::string& name, const std::string& frame, float r, float g, float b)
{
	visualization_msgs::MarkerArray markerArray;

	for(std::size_t i=0; i<pose.size();i++)
	{
		markerArray.markers.push_back (createMarker(frame, name, i, pose[i].position.x, pose[i].position.y, pose[i].position.z, 1.0, 1.0, 0.0, 1.0));
	}
	return markerArray;
}


std::vector<geometry_msgs::Pose> GetContactPoints::getRegions(cv::Mat mapImage, const double& regionLength, const double& regionWidth, const std::string& destination_frame, const std::string& original_frame)
{
	geometry_msgs::Pose pose;

	pose.position.x = 0;
	pose.position.y = 0;
	pose.position.z = 0;

	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1.0;

	//************** recalc the area witch has to be taken from the map
	geometry_msgs::Pose pose1;

	geometry_msgs::Pose pose2;

	pose1 = pose;
	pose2 = pose;

	pose1.position.x = regionLength/2;
	pose2.position.x = -regionLength/2;

	pose1 = tfTransform(pose1, destination_frame, original_frame);
	pose2 = tfTransform(pose2, destination_frame, original_frame);

	double regionMapLength = clcDistanz(pose1,pose2);

	pose1 = pose;
	pose2 = pose;

	pose1.position.x = regionWidth/2;
	pose2.position.x = -regionWidth/2;

	pose1 = tfTransform(pose1, destination_frame, original_frame);
	pose2 = tfTransform(pose2, destination_frame, original_frame);

	double regionMapWidth = clcDistanz(pose1,pose2);


	geometry_msgs::Pose nextPose = tfTransform(pose, destination_frame, original_frame);

	cv::Mat flipperMap;

	flipperMap = getCropedImage(nextPose, mapImage, regionMapWidth, regionMapLength);

	return getPosesFromImage(flipperMap, nextPose, original_frame, destination_frame);
}

double GetContactPoints::clcDistanz(const geometry_msgs::Pose& pose1,const geometry_msgs::Pose& pose2)
{
	double x = pose1.position.x - pose2.position.x; //calculating number to square in next step
	double y = pose1.position.y - pose2.position.y;
	double dist;

	dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
	dist = sqrt(dist);

	return dist;

}



cv::Mat GetContactPoints::getCropedImage(geometry_msgs::Pose& pose, cv::Mat mapImage, double rectSizeX, double rectSizeY)
{
    mapSizeX = mapImage.cols;
    mapSizeY = mapImage.rows;

   	int x = mapSizeX/2 - pose.position.y/resultion;
   	int y = mapSizeY/2 - pose.position.x/resultion;


	int widthX = rectSizeX/resultion;
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
/*
    cv::Mat copy = mapImage.clone();

    DrawRotatedRectangle(copy,rect );
    imshow("wadw", copy);
    waitKey(1);

    imshow("awdawd", cropped);
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


std::vector<geometry_msgs::Pose> GetContactPoints::getPosesFromImage(cv::Mat flipperMaps, const geometry_msgs::Pose& nextPose, const std::string& destination_frame,const std::string& original_frame)
{
	std::vector<geometry_msgs::Pose> poses;

	geometry_msgs::Pose pose;
	geometry_msgs::Pose tranformedPose;

	tf::Quaternion quat;
	quat.setRPY( 0, 0, 0);
	pose.orientation.x = quat.x();
	pose.orientation.y = quat.y();
	pose.orientation.z = quat.z();
	pose.orientation.w = quat.w();

	//int x = mapSizeX/2 - nextRobotPose.position.y/resultion;
   	//int y = mapSizeY/2 - nextRobotPose.position.x/resultion;


	double cropLengthX = flipperMaps.rows*resultion;
	double cropLengthY = flipperMaps.cols*resultion;

	cv::Mat bwsrc;
	cv::Mat src;
	flipperMaps.copyTo(src);

	float theta= 0;
	double value = 0;
	for(int i=0; i<flipperMaps.rows; i++)
	{
	    for(int j=0; j<flipperMaps.cols; j++)
	    {
	    	quat.setX(nextPose.orientation.x);
	    	quat.setY(nextPose.orientation.y);
	    	quat.setZ(nextPose.orientation.z);
	    	quat.setW(nextPose.orientation.w);

	    	theta = tf::getYaw(quat);

	    	pose.position.x = nextPose.position.x + (cropLengthX/2 - cropLengthX/(flipperMaps.rows*2) - i*resultion);
	    	pose.position.y = nextPose.position.y + (cropLengthY/2 - cropLengthY/(flipperMaps.cols*2) - j*resultion);
			value = src.at<char >(i,j);

	    	pose.position.z = value*0.0043;

	    	pose = rotate_point(pose, theta, nextPose);

	    	tranformedPose = tfTransform(pose, destination_frame, original_frame);

	    	poses.push_back(tranformedPose);

	    }
	}
	return poses;
}


std::vector<geometry_msgs::Pose> GetContactPoints::clcNewPoses(const std::vector<geometry_msgs::Pose>& poses, tf2::Quaternion q, const std::string& baseFrame, const std::string& flipperFrame)
{
	std::vector<geometry_msgs::Pose> transformedPose = transformPose(poses, baseFrame, flipperFrame);

	tf2::Quaternion q_prime = q.inverse();
	tf2::Quaternion p;
	p.setX(0);
	tf2::Quaternion p_prime;

	std::vector<geometry_msgs::Pose> newPoses;
	geometry_msgs::Pose pose_prime;
	for(auto pose : transformedPose)
	{
		//ROS_INFO (" pose:\t x = [%7.3lf], y = [%7.3lf], z = [%7.3lf]", pose.position.x, pose.position.y, pose.position.z);
		p.setY(pose.position.x);
		p.setZ(pose.position.y);
		p.setW(pose.position.z);

		p_prime=q*p*q_prime;
		pose_prime.position.x = p_prime.getY();
		pose_prime.position.y = p_prime.getZ();
		pose_prime.position.z = p_prime.getW();
		newPoses.push_back(pose_prime);
		//ROS_INFO (" pose_prime:\t x = [%7.3lf], y = [%7.3lf], z = [%7.3lf]", pose_prime.position.x, pose_prime.position.y, pose_prime.position.z);
	}

	double maxZ = 0;
	for(auto pose : newPoses)
	{
		if(pose.position.z > maxZ)
		{
			maxZ = pose.position.z;
		}
	}
	//ROS_INFO (" newPoses[i]:\t x = [%7.3lf], y = [%7.3lf], z = [%7.3lf]", newPoses[i].position.x, newPoses[i].position.y, newPoses[i].position.z);

	for(std::size_t i=0; i < newPoses.size(); i++)
	{
		newPoses[i].position.z = newPoses[i].position.z - maxZ;
		//ROS_INFO (" newPoses[i]:\t x = [%7.3lf], y = [%7.3lf], z = [%7.3lf]", newPoses[i].position.x, newPoses[i].position.y, newPoses[i].position.z);
	}
	newPoses = transformPose(newPoses, flipperFrame,baseFrame);

	return newPoses;
}
geometry_msgs::Pose GetContactPoints::rotate_point(geometry_msgs::Pose pPose ,const float& theta,const geometry_msgs::Pose& oPose)
{
	geometry_msgs::Pose newPose = pPose;
	newPose.position.x = cos(theta) * (pPose.position.x-oPose.position.x) - sin(theta) * (pPose.position.y-oPose.position.y) + oPose.position.x;
	newPose.position.y = sin(theta) * (pPose.position.x-oPose.position.x) + cos(theta) * (pPose.position.y-oPose.position.y) + oPose.position.y;
	return newPose;
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



