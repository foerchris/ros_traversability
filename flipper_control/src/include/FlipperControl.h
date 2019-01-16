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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "GetContactPoints.h"

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

	void publishAngles (flipperAngles robotFlipperAngles);

	void SequenceControl(cv::Mat mapImage);





	ros::Publisher frontFlipperAngleDesiredPub;
	ros::Publisher rearFlipperAngleDesiredPub;
	ros::Publisher markerPublisher;

	ros::NodeHandle nodeHandle_;


	ros::Timer msg_timer ;

	std::string tf_prefix;

	// Obeject declarations
	GetContactPoints getContactPoints;


	cv::Mat globalMapImage;

	// true if
	bool mapImageSet;



};


#endif /* ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_FLIPPERCONTROL_H_ */
