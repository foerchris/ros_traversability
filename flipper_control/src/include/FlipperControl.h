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

	void publishAngles (flipperAngles robotFlipperAngles);

	void SequenceControl(cv::Mat mapImage);

	tf2::Quaternion groundPlane(cv::Mat image,const geometry_msgs::Twist& velocitiy_robot, const double& delta_t);

	double flipperEval(const std::string& flipper, cv::Mat image, const tf2::Quaternion& quat, int frontRear);
	double returnBiggerVel(const double& vel1, const double& vel2);

	void odomCallback (const nav_msgs::OdometryConstPtr& odomMsg);

	void publishDesiredRobotPose (tf2::Quaternion quat);

	ros::Publisher frontFlipperAngleDesiredPub;
	ros::Publisher rearFlipperAngleDesiredPub;
	ros::Publisher markerPublisher;
	ros::Publisher desired_robot_pose_pub;

	ros::Subscriber	odomSub;
	ros::NodeHandle nodeHandle_;


	ros::Timer msg_timer ;

	std::string tf_prefix;

	// Obeject declarations
	GetContactPoints getContactPoints;
	FitPlane fitPlane;
	CalcFlipperAngles calcFlipperAngles;

	cv::Mat globalMapImage;

	geometry_msgs::Twist currentVelocity;

	ros::Time start_time;
	double delta_t;

	// true if
	bool mapImageSet;



};


#endif /* ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_FLIPPERCONTROL_H_ */
