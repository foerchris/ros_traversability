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

// Tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

struct minMaxFlipperVel {
	double min;
	double max;
	cv::Point min_loc;
	cv::Point max_loc;
};

struct flipperContactPointsAngles {
	std::vector<geometry_msgs::Pose> pose;
	std::vector<double> phi1;
	std::vector<double> phi2;
	std::vector<double> phiContact;
	std::vector<double> flipperAngle;
};


struct flipperAngles {
	double flipperAngleFront;
	double flipperAngleRear;
};

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

	// Transfrom pose relative to map to pose relative to robot
	geometry_msgs::Pose flipperToMapTransform(const geometry_msgs::Pose& pose,const std::string& tfID);
	geometry_msgs::Pose mapToFlipperTransform(const geometry_msgs::Pose& pose,const std::string& tfID);
	flipperContactPointsAngles clcContactAngles(const std::vector<geometry_msgs::Pose>& values, std::string flipperFrame);
	std::vector<geometry_msgs::Pose> getContactPoints(cv::Mat flipperMaps);

	void SequenceControl(cv::Mat mapImage);
	std::vector<cv::Mat> getFlipperRegions(cv::Mat mapImage);
	cv::Mat getCropedImage(geometry_msgs::Pose& pose, cv::Mat mapImage);
	std::vector<minMaxFlipperVel> clcMinMaxVels(std::vector<cv::Mat> flipperMaps);
	void clcFlipperAngles(const std::vector<minMaxFlipperVel>& minMaxVel, flipperAngles& robotFlipperAngles);

	std::string BASE_FRAME;
	std::string ODOM_FRAME;
	std::string MAP_FRAME;
	ros::NodeHandle nodeHandle_;

	std::unique_ptr<tf::TransformListener> tfListener;
	tf::StampedTransform transform;

	std::string tf_prefix;

	ros::Publisher frontFlipperAngleDesiredPub;
	ros::Publisher rearFlipperAngleDesiredPub;

	void publishAngles (flipperAngles robotFlipperAngles);

	// Robot parameter
	double R;		// Radius of the wheel of the robot body
	double r;		// Radius of the wheel on the end of the flipper
	double L;		// Wheel distance between the robot wheel axis and the wheel axis on the end of the flipper
	//double d;		// Distance between body wheel axis and "downside of flipper wheel"
	double theta;	// Angle of distance (see above) and flipper wheel radius
	double dThreshold;

	/////*********************** the image variables
	double resultion;
	double xLength  ;
	double yLength  ;
	double mapSizeX;
	double mapSizeY;
};


#endif /* ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_FLIPPERCONTROL_H_ */
