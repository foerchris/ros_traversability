/*
 * FlipperControl.h
 *
 *  Created on: 12.12.2018
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_FLIPPERCONTROL_SRC_GETCONTACTPOINTS_H_
#define ROS_ROBOCUP_FLIPPERCONTROL_SRC_GETCONTACTPOINTS_H_

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


class GetContactPoints
{
	public:
	/*!
	 * Constructor.
	 * @param nodeHandle the ROS node handle.
	 */
	GetContactPoints();

	/*!
	 * Destructor.
	 */
	virtual ~GetContactPoints();

	// return an vector of the segmented tracks and flipper regions
	std::vector<cv::Mat> getFlipperRegions(cv::Mat mapImage);

	// returns the the robot ground region
	cv::Mat getRobotRegions(cv::Mat mapImage);


	std::vector<geometry_msgs::Pose> transformPose(const std::vector<geometry_msgs::Pose>& poses,const std::string& destination_frame,const std::string& original_frame);

	// tf transformation method
	geometry_msgs::Pose tfTransform(const geometry_msgs::Pose& pose,const std::string& destination_frame,const std::string& original_frame);

	// cropes the image with an rotated rect according to the position and orientation of the robot inside the map
	cv::Mat getCropedImage(geometry_msgs::Pose& pose, cv::Mat mapImage,double rectSizeX, double rectSizeY);

	std::vector<geometry_msgs::Pose> procGroundImage(cv::Mat flipperMaps);


	// draws the rotated rectangle for the flipper region into the image for debuging
	void DrawRotatedRectangle(cv::Mat& image, cv::RotatedRect rotatedRectangle);



	// process from image to koordinates
	std::vector<geometry_msgs::Pose> procFlipperMaps(cv::Mat flipperMaps, std::string flipperFrame);

	// display the each point through a marker array
	visualization_msgs::MarkerArray creatMarkerArrayFlipperPoints(const std::vector<geometry_msgs::Pose>& pose, const std::string& name, const std::string& frame, float r, float g, float b);

	visualization_msgs::Marker createMarker (std::string ns, int id, double x, double y,  double r, double g, double b, double a);


	private:
	// *********** definitions for tf tranform
	std::string BASE_FRAME;
	std::string ODOM_FRAME;
	std::string MAP_FRAME;
	ros::NodeHandle nodeHandle_;

	std::unique_ptr<tf::TransformListener> tfListener;
	tf::StampedTransform transform;

	std::string tf_prefix;

	// Robot parameter
	double R;		// Radius of the wheel of the robot body
	double r;		// Radius of the wheel on the end of the flipper
	double L;		// Wheel distance between the robot wheel axis and the wheel axis on the end of the flipper
	//double d;		// Distance between body wheel axis and "downside of flipper wheel"
	double theta;	// Angle of distance (see above) and flipper wheel radius
	double dThreshold;
	double trackLength;

	// *********** the image variables
	double resultion;
	double xLength  ;
	double yLength  ;
	double mapSizeX;
	double mapSizeY;
	double TracksBaseLinkDist;
	double FlipperTrackLength;
	double cropeMapLength;
};


#endif /* ROS_ROBOCUP_FLIPPERCONTROL_SRC_GETCONTACTPOINTS_H_ */
