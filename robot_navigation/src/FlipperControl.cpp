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
	tfListener = std::unique_ptr<tf::TransformListener> (new tf::TransformListener);

	markerPublisher = nodeHandle_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 20);


	frontFlipperAngleDesiredPub = nodeHandle_.advertise < std_msgs::Float64 > ("flipper_front_controller/command", 1);
	rearFlipperAngleDesiredPub 	= nodeHandle_.advertise < std_msgs::Float64 > ("flipper_rear_controller/command", 1);

	msg_timer = nodeHandle_.createTimer(ros::Duration(0.1), boost::bind (&FlipperControl::FlipperSequenzCallback, this, _1));


	tf_prefix = "//GETjag";
	tf_prefix = ros::this_node::getNamespace();

	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	BASE_FRAME = tf_prefix + "/base_link";
	ODOM_FRAME = tf_prefix + "/odom";
	MAP_FRAME = tf_prefix + "/map";

	mapImageSet = false;

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
	//yLength  = 0.2575;
//	yLength = L+r;
	xLength = L+r;
	trackLength = 0.5;
	FlipperTrackLength = 2*(xLength + R) + trackLength;
	TracksBaseLinkDist = 0.275;

}

FlipperControl::~FlipperControl()
{
	nodeHandle_.shutdown();
}


visualization_msgs::Marker FlipperControl::createMarker (std::string ns, int id, double x, double y,  double r, double g, double b, double a)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = tf_prefix+"/map";
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

	//SequenceControl(globalMapImage);

	mapImageSet = true;
}


void FlipperControl::FlipperSequenzCallback(const ros::TimerEvent& event)
{
	if(mapImageSet)
	{
		SequenceControl(globalMapImage);
		//imshow("globalMapImage",globalMapImage);
		//waitKey(1);
	}
}

void FlipperControl::SequenceControl(cv::Mat mapImage)
{
	cv::Point min_loc, max_loc;

	std::vector<cv::Mat> flipperMaps;
	std::vector<cv::Mat> TracksPlusFlipperImage;

	flipperMaps = getFlipperRegions(mapImage);

	TracksPlusFlipperImage = getTrackedRegions(mapImage, "/static_base_link");

	std::vector<geometry_msgs::Pose> tracksContactPoints;

	tracksContactPoints = procTrackMaps(TracksPlusFlipperImage[0], 1, "/static_base_link");
	displayFlipperPoints(tracksContactPoints, "tacks_left", "/static_base_link");
	geometry_msgs::Pose meanPoseTrackLeft = clcMean(tracksContactPoints);


	tracksContactPoints = procTrackMaps(TracksPlusFlipperImage[1], -1, "/static_base_link");
	displayFlipperPoints(tracksContactPoints, "tracks_right","/static_base_link");
	geometry_msgs::Pose meanPoseTrackRight = clcMean(tracksContactPoints);


	//***************************
	std::vector<geometry_msgs::Pose> contactPoints;

	contactPoints = getContactPoints(flipperMaps[0], "/static_flipper_front_left");
	displayFlipperPoints(contactPoints, "static_flipper_front_left", "/static_flipper_front_left");
	flipperContactPointsAngles flipperFrontLeft;
	flipperFrontLeft = clcContactAngles(contactPoints, "/static_flipper_front_left");
	geometry_msgs::Pose displayPose;


	contactPoints = getContactPoints(flipperMaps[1], "/static_flipper_front_right");
	displayFlipperPoints(contactPoints, "static_flipper_front_right", "/static_flipper_front_right");
	flipperContactPointsAngles flipperFrontRight;
	flipperFrontRight = clcContactAngles(contactPoints, "/static_flipper_front_right");


	contactPoints = getContactPoints(flipperMaps[2], "/static_flipper_rear_left");
	displayFlipperPoints(contactPoints, "static_flipper_rear_left","/static_flipper_rear_left");
	flipperContactPointsAngles flipperRearLeft;
	flipperRearLeft = clcContactAngles(contactPoints, "/static_flipper_rear_left");


	contactPoints = getContactPoints(flipperMaps[3], "/static_flipper_rear_right");
	displayFlipperPoints(contactPoints, "static_flipper_rear_right", "/static_flipper_rear_right");
	flipperContactPointsAngles flipperRearRight;
	flipperRearRight = clcContactAngles(contactPoints, "/static_flipper_rear_right");


	flipperAngles robotFlipperAngles;

	robotFlipperAngles.flipperAngleFront = clcFlipperAngles(flipperFrontLeft, flipperFrontRight);
	robotFlipperAngles.flipperAngleRear= clcFlipperAngles(flipperRearLeft, flipperRearRight);

	//ROS_INFO("robotFlipperAngles:  flipperAngleFront = %lf, flipperAngleRear = %lf\n", robotFlipperAngles.flipperAngleFront, robotFlipperAngles.flipperAngleRear);

	//publishAngles(robotFlipperAngles);
}

void FlipperControl::displayFlipperPoints(const std::vector<geometry_msgs::Pose>& pose, const std::string& name, const std::string& frame)
{
	geometry_msgs::Pose displayPose;
	visualization_msgs::MarkerArray markerArray;

	for(std::size_t i=0; i<pose.size();i++)
	{

		displayPose = flipperToMapTransform(pose[i], tf_prefix + frame);

		markerArray.markers.push_back (createMarker(tf_prefix + name, i, displayPose.position.x, displayPose.position.y, 1.0, 1.0, 0.0, 1.0));
	}
	markerPublisher.publish(markerArray);
}

geometry_msgs::Pose FlipperControl::clcDesiredPose(const geometry_msgs::Pose& meanPose)
{
	geometry_msgs::Pose desiredPose;

	double aplhaXY = meanPose.position.x *  meanPose.position.y

	double a = ()
	return desiredPose;
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

		d= sqrt(pow(x, 2)+ pow(z, 2));

		if(flipperFrame == "/static_flipper_front_left")
		{
			ROS_INFO("contactPoints: x = %lf, z = %lf", values[i].position.x, values[i].position.z);

			ROS_INFO("flipperFrontLeft: dThreshold = %lf, d = %lf, x = %lf, z = %lf", dThreshold, d, x, z);
		}

		if(flipperFrame == "/static_flipper_rear_left")
		{
			ROS_INFO("contactPoints: x = %lf, z = %lf", values[i].position.x, values[i].position.z);

			ROS_INFO("flipperRearLeft: dThreshold = %lf, d = %lf, x = %lf, z = %lf", dThreshold, d, x, z);
		}

		if(d<= dThreshold)
		{
			phi1 = atan(z/x);
			phi2 = asin(R/d);
			phiContact = phi1 + phi2;

			//********************************************************** nochmal nachrechnen **********************************************************
			// flipperAngle = phi1 - asin((R-r)/L);		// wenn mittel Linie parallel zur x achse
			flipperAngle = phiContact; 					// wenn untere flipper linie parallel zur x achse
		}
		else if(d > dThreshold && d < xLength)
		{
			phi1 = asin((pow(d,2) + pow(L,2) - pow(r,2))/ (2*L*d)) - atan(x/z);
			phi2 = asin((R-r)/L);
			phiContact = phi1 + phi2;

			//********************************************************** nochmal nachrechnen **********************************************************
			//flipperAngle = phi1;						// wenn mittel Linie parallel zur x achse
			flipperAngle = phiContact; 					// wenn untere flipper linie parallel zur x achse

		}
		else
		{
			phi1 = -M_PI/2;
			phi2 = -M_PI/2;
			phiContact = -M_PI/2;

			flipperAngle= -M_PI/2;
		}
		if(flipperFrame == "/static_flipper_front_left")
		{
			ROS_INFO("flipperFrontLeft: phi1 = %lf, phi2 = %lf, phiContact = %lf, flipperAngle = %lf", phi1*R2D, phi2*R2D, phiContact*R2D, flipperAngle*R2D);
		}
		if(flipperFrame == "/static_flipper_rear_left")
		{
			ROS_INFO("flipperRearLeft: phi1 = %lf, phi2 = %lf, phiContact = %lf, flipperAngle = %lf", phi1*R2D, phi2*R2D, phiContact*R2D, flipperAngle*R2D);
		}
		robotFlipperAngles.pose.push_back(point);
		robotFlipperAngles.phi1.push_back(phi1);
		robotFlipperAngles.phi2.push_back(phi2);
		robotFlipperAngles.phiContact.push_back(phiContact);
		robotFlipperAngles.flipperAngle.push_back(flipperAngle);

	}
	return robotFlipperAngles;
}

double FlipperControl::clcFlipperAngles(flipperContactPointsAngles flipperLeft, flipperContactPointsAngles flipperRight)
{

	auto it1 = max_element(std::begin(flipperLeft.phiContact), std::end(flipperLeft.phiContact));
	int index1 = std::distance(flipperLeft.phiContact.begin(), it1);

	auto it2 = max_element(std::begin(flipperRight.phiContact), std::end(flipperRight.phiContact));
	int index2 = std::distance(flipperRight.phiContact.begin(), it2);

	if(flipperLeft.phiContact[index1] > flipperRight.phiContact[index2])
	{
		return flipperLeft.flipperAngle[index1];
	}
	else
	{
		return flipperRight.flipperAngle[index2];

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

std::vector<geometry_msgs::Pose> FlipperControl::getContactPoints(cv::Mat flipperMaps, std::string flipperFrame)
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


	for(int i=0; i<flipperMaps.rows; i++)
	{
	    for(int j=0; j<flipperMaps.cols; j++)
	    {
	    	pose.position.x = (xLength - xLength/(flipperMaps.rows*2)) + i*resultion;
	    	pose.position.y = j*resultion;

	    	pose.position.z = flipperMaps.at<cv::Vec2b>(i, j)[0]*0.0043;
			flipperPose.position.z = pose.position.z;
			flipperPose = mapToFlipperTransform(flipperPose, tf_prefix + flipperFrame);
			pose.position.z = flipperPose.position.z;
	    	poses.push_back(pose);
	    }
	}
	return poses;
}

std::vector<geometry_msgs::Pose> FlipperControl::procTrackMaps(cv::Mat flipperMaps, const int& flipperLeftRight, std::string flipperFrame)
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

    std::cout<<"flipperMaps.size"<<flipperMaps.rows*flipperMaps.cols<<std::endl;

	for(int i=0; i<flipperMaps.rows; i++)
	{
	    for(int j=0; j<flipperMaps.cols; j++)
	    {
	    	pose.position.x = (FlipperTrackLength/2  - FlipperTrackLength/(flipperMaps.rows*2)) - i*resultion;
	    	pose.position.y = j*resultion + flipperLeftRight*TracksBaseLinkDist;

	    	pose.position.z = flipperMaps.at<cv::Vec2b>(i, j)[0]*0.0043;
			flipperPose.position.z = pose.position.z;
			flipperPose = mapToFlipperTransform(flipperPose, tf_prefix + flipperFrame);
			pose.position.z = flipperPose.position.z;
	    	poses.push_back(pose);
	    }
	}

//	for(int i=0; i<poses.size();i++)
	//{
	  //  std::cout<<"poses"<<poses[i].position<<std::endl;

	//}

	return poses;
}

geometry_msgs::Pose FlipperControl::clcMean(std::vector<geometry_msgs::Pose> poses)
{
	geometry_msgs::Pose mean;
	mean.position.x = 0;
	mean.position.y = 0;
	mean.position.z = 0;
	int N = poses.size();
	for(int i=0; i<N;i++)
	{
		mean.position.x = mean.position.x + poses[i].position.x;
		mean.position.y = mean.position.y + poses[i].position.y;
		mean.position.z = mean.position.z + poses[i].position.z;
	}
	mean.position.x = mean.position.x/N;
	mean.position.y = mean.position.y/N;
	mean.position.z = mean.position.z/N;

	return mean;
}

// Include center point of your rectangle, size of your rectangle and the degrees of rotation
void FlipperControl::DrawRotatedRectangle(cv::Mat& image, cv::RotatedRect rotatedRectangle)
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

std::vector<cv::Mat> FlipperControl::getFlipperRegions(cv::Mat mapImage)
{
	geometry_msgs::Pose pose;
	pose.position.x =  xLength/2;
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

std::vector<cv::Mat> FlipperControl::getTrackedRegions(cv::Mat mapImage, const std::string& flipperFrame)
{

	geometry_msgs::Pose pose;
	pose.position.x =  0.0;
	pose.position.y = TracksBaseLinkDist;
	pose.position.z = 0.0;
	tf::Quaternion quat;
	quat.setRPY( 0, 0, 0);

	pose.orientation.x = quat.x();
	pose.orientation.y = quat.y();
	pose.orientation.z = quat.z();
	pose.orientation.w = quat.w();

	std::vector<cv::Mat> flipperMaps;
	cv::Mat flipperMap;

	geometry_msgs::Pose flipperPose = flipperToMapTransform(pose, tf_prefix + flipperFrame);
	flipperMap = getTracksImage(flipperPose, mapImage);
	flipperMaps.push_back(flipperMap);

	pose.position.y = -TracksBaseLinkDist;
	flipperPose = flipperToMapTransform(pose, tf_prefix + flipperFrame);
	flipperMap = getTracksImage(flipperPose, mapImage);
	flipperMaps.push_back(flipperMap);

	return flipperMaps;
}

cv::Mat FlipperControl::getTracksImage(geometry_msgs::Pose& pose, cv::Mat mapImage)
{
    mapSizeX = mapImage.cols;
    mapSizeY = mapImage.rows;

   	int x = mapSizeX/2 - pose.position.y/resultion;
   	int y = mapSizeY/2 - pose.position.x/resultion;

	int widthX = yLength/resultion;
	int widthY = FlipperTrackLength/resultion;

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

    /*cv::Mat drawRect;
    mapImage.copyTo(drawRect);
	DrawRotatedRectangle(drawRect, rect);
	imshow("output",drawRect);
	waitKey(1);*/
    std::cout<<"cropped"<<cropped<<std::endl;

	return cropped;
}
cv::Mat FlipperControl::getCropedImage(geometry_msgs::Pose& pose, cv::Mat mapImage)
{


    mapSizeX = mapImage.cols;
    mapSizeY = mapImage.rows;

   	int x = mapSizeX/2 - pose.position.y/resultion;
   	int y = mapSizeY/2 - pose.position.x/resultion;

	int widthX = yLength/resultion;
	int widthY =  xLength/resultion;

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

    /*cv::Mat drawRect;
    mapImage.copyTo(drawRect);
	DrawRotatedRectangle(drawRect, rect);
	imshow("output2",drawRect);
	waitKey(1);*/

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
