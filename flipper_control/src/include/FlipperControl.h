/*
 * FlipperControl.h
 *
 *  Created on: 12.12.2018
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_FLIPPERCONTROL_H_
#define ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_FLIPPERCONTROL_H_

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "GetContactPoints.h"
#include "CalcFlipperAngles.h"
#include "FitPlane.h"

class FlipperControl
{
	public:
	/*!
	 * Constructor.
	 * @param nodeHandle the ROS node handle.
	 */
	FlipperControl(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~FlipperControl();

	private:

	void MapImageCallback(const sensor_msgs::ImageConstPtr& msg);

	void FlipperSequenzCallback(const ros::TimerEvent& event);

	void SequenceControl(cv::Mat mapImage);

	tf2::Quaternion groundPlane(cv::Mat image);

	double flipperRegion(cv::Mat image,const tf2::Quaternion& quat, const std::string& flipperFrame);

	ros::Publisher markerPublisher;

	ros::NodeHandle nodeHandle_;

	ros::Timer msg_timer ;

	std::string tf_prefix;
	//**************** tf frames
	std::string BASE_FRAME;
	std::string NEXT_BASE_FRAME;
	std::string MAP_FRAME;
	std::string ODOM_FRAME;
	std::string FLIPPER_FRONT_LEFT_FRAME;
	std::string FLIPPER_FRONT_RIGHT_FRAME;
	std::string FLIPPER_REAR_LEFT_FRAME;
	std::string FLIPPER_REAR_RIGHT_FRAME;

	// Obeject declarations
	GetContactPoints getContactPoints;
	FitPlane fitPlane;
	CalcFlipperAngles calcFlipperAngles;

	cv::Mat globalMapImage;

	// true if
	bool mapImageSet;



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
	double flipperWidth;
	double yLength  ;
	double flipperLength;
	double mapSizeX;
	double mapSizeY;
	double TracksBaseLinkDist;
	double FlipperTrackLength;
	double cropeMapLength;



};


#endif /* ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_FLIPPERCONTROL_H_ */
