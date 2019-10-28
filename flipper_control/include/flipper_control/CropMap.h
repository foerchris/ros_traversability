/*
 * FlipperControl.h
 *
 *  Created on: 12.12.2018
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_FLIPPERCONTROL_SRC_CropMap_H_
#define ROS_ROBOCUP_FLIPPERCONTROL_SRC_CropMap_H_

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <nav_msgs/Odometry.h>

// Tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// marker array
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

struct Vector3
{
    float x, y, z;
};
class CropMap
{
	public:
	/*!
	 * Constructor.
	 * @param nodeHandle the ROS node handle.
	 */
	CropMap();

	/*!
	 * Destructor.
	 */
	virtual ~CropMap();


	// returns the a region according to the  width and length from the map given its coordinat frame
	std::vector<geometry_msgs::Pose> getRegions(cv::Mat mapImage,const double& regionLength, const double& regionWidth, const std::string& destination_frame, const std::string& original_frame);
	cv::Mat getRobotGroundImage(cv::Mat mapImage, const double& regionLength, const double& regionWidth, const std::string& destination_frame, const std::string& original_frame);


	// cropes the image with an rotated rect according to the position and orientation of the robot inside the map
	cv::Mat getCropedImage(geometry_msgs::Pose& pose, cv::Mat mapImage,double rectSizeX, double rectSizeY);

	std::vector<geometry_msgs::Pose> getPosesFromImage(cv::Mat flipperMaps, const geometry_msgs::Pose& nextPose, const std::string& destination_frame,const std::string& original_frame);


	double clcMaxZ(const std::vector<geometry_msgs::Pose>& poses, tf2::Quaternion q, const double& trackLength);
	std::vector<geometry_msgs::Pose> clcNewFlipperPoses(const std::vector<geometry_msgs::Pose>& poses, tf2::Quaternion q, const double maxZ, const std::string& baseFrame, const std::string& flipperFrame, const std::string& flipperRegionFrame);


	//****************************************** helper functions
	void setConstants(const double& imageResultion);


	std::vector<geometry_msgs::Pose> transformPose(const std::vector<geometry_msgs::Pose>& poses,const std::string& destination_frame,const std::string& original_frame);

	// tf transformation method
	geometry_msgs::Pose tfTransform(const geometry_msgs::Pose& pose,const std::string& destination_frame,const std::string& original_frame);

	// draws the rotated rectangle for the flipper region into the image for debuging
	void DrawRotatedRectangle(cv::Mat& image, cv::RotatedRect rotatedRectangle);

	// display the each point through a marker array
	visualization_msgs::MarkerArray creatMarkerArrayFlipperPoints(const std::vector<geometry_msgs::Pose>& pose, const std::string& name, const std::string& frame, float r, float g, float b);
	visualization_msgs::Marker createMarker (const std::string& tfFrame, const std::string& ns,const int& id,const double& x,const double& y,const double& z,const  double& r,const double& g,const double& b,const double& a);

	visualization_msgs::MarkerArray creatCubeMarkerArrayFlipperPoints(const geometry_msgs::Pose& pose, const std::string& name, const std::string& frame, float r, float g, float b);
	visualization_msgs::Marker createCubeMarker (const std::string& tfFrame, const std::string& ns,const int& id,const geometry_msgs::Pose& pose,const  double& r,const double& g,const double& b,const double& a);


	tf2::Quaternion getDestQuat(tf2::Quaternion q, const std::string& destination_frame, const std::string& original_frame, const double& setRoll, const double& setPitch);


	visualization_msgs::MarkerArray creatMarkerArrayFlipperLine(const std::vector<geometry_msgs::Pose>& pose, const std::string& name, const std::string& frame, float r, float g, float b);

	private:
	// *********** definitions for tf tranform
	double clcDistanz(const geometry_msgs::Pose& pose1,const geometry_msgs::Pose& pose2);
	geometry_msgs::Pose rotate_point(geometry_msgs::Pose pPose ,const float& theta,const geometry_msgs::Pose& oPose);
	cv::Mat getCropedImage16US1(geometry_msgs::Pose& pose, cv::Mat mapImage, double rectSizeX, double rectSizeY);

	std::vector<geometry_msgs::Pose> clcNewPoses(const std::vector<geometry_msgs::Pose>& poses, tf2::Quaternion q);


	geometry_msgs::Pose clcQuternion(const geometry_msgs::Pose& pose,const tf2::Quaternion& q);


	visualization_msgs::Marker createLineMarker (const std::string& tfFrame, const std::string& ns,const int& id, const std::vector<geometry_msgs::Pose>& poses,const  double& r,const double& g,const double& b,const double& a);
	std::unique_ptr<tf::TransformListener> tfListener;
	tf::StampedTransform transform;

	std::string tf_prefix;

	//*****
	double resultion;

	double mapSizeX;
	double mapSizeY;


};


#endif /* ROS_ROBOCUP_FLIPPERCONTROL_SRC_CropMap_H_ */
