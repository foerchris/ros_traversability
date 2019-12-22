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
#include <flipper_control/CropMap.h>
#include <flipper_control/FlipperAngles.h>
#include <flipper_control/NESM.h>
#include <flipper_control/FitPlane.h>

#include <dynamic_reconfigure/server.h>
#include <flipper_control/FlipperControlConfig.h>

struct NESMValues {
	double S_NE_frontRearLeft;
	double S_NE_rearFrontLeft;

	double S_NE_frontRearRight;
	double S_NE_rearFrontRight;

	double S_NE_leftRightFront;
	double S_NE_rightLeftFront;

	double S_NE_leftRightRear;
	double S_NE_rightLeftRear;
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

	/*!
	 * callback to subscribe elevation map
	 * @param msg
	 */
	void MapImageCallback(const sensor_msgs::ImageConstPtr& msg);

	/*!
	 * timer to execute flipper calculation
	 * @param event
	 */
	void FlipperSequenzCallback(const ros::TimerEvent& event);

	/*!
	 * Sequence to calculate flippers
	 * @param mapImage
	 */
	void SequenceControl(cv::Mat mapImage);

	/*!
	 * calculate the hyperplane and max z values
	 * @param image; image of the elevation map
	 * @param maxZValue; max z value
	 * @param fittedPlane; hyperplane
	 * @return
	 */
	tf2::Quaternion groundPlane(cv::Mat image, double* maxZValue, FittedPlane* fittedPlane);

	/*!
	 * croped region for the flipper
	 * @param image; image of the elevation map
	 * @param quat; quternion from hyperplane
	 * @param maxZ; max z value
	 * @param flipperFrame; frame of flipper
	 * @param flipperRegionFrame
	 * @return calculated flipper angles
	 */
	MaxFlipperContactPointsAngles flipperRegion(cv::Mat image,const tf2::Quaternion& quat, const double& maxZ, const std::string& flipperFrame, const std::string& flipperRegionFrame);

	///*********************** stability analysis for all possibility's

	/*!
	 * calculate stability analysis
	 * @param g1
	 * @param g2
	 * @param c
	 * @param frame1
	 * @param frame2
	 * @param rotatePitch
	 * @param rotationDirection
	 * @return
	 */
	double stabilityAnalysis(const geometry_msgs::Pose& g1,const geometry_msgs::Pose& g2, const geometry_msgs::Pose& c, const std::string& frame1, const std::string& frame2,  const bool& rotatePitch, const int& rotationDirection);

	/*!
	 * check if the stability criteria is satisfied
	 * @param frontLeft; nesm front, from right to left
	 * @param frontRight; nesm front, from left to right
	 * @param rearLeft; nesm rear, from right to left
	 * @param rearRight; nesm rear, from left to right
	 * @return return 0 if robot is stable
	 */
	int cheakNESM(MaxFlipperContactPointsAngles frontLeft, MaxFlipperContactPointsAngles frontRight, MaxFlipperContactPointsAngles rearLeft, MaxFlipperContactPointsAngles rearRight);


	/*!
	 * return bigger value
	 * @param vel1
	 * @param vel2
	 * @return bigger value
	 */
	double returnBiggerVel(const double& vel1, const double& vel2);

	/*!
	 * publish the flipper angle for the robot
	 * @param robotFlipperAngles
	 */
	void publishAngles (RobotFlipperAngles robotFlipperAngles);

	/*!
	 * add two poses
	 * @param pose1
	 * @param pose2
	 * @return result
	 */
	geometry_msgs::Pose addPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);

	/*!
	 * subtract two poses
	 * @param pose1
	 * @param pose2
	 * @return result
	 */
	geometry_msgs::Pose subPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);

	/*!
	 * dynamic reconfigure callback
	 * @param config
	 * @param level
	 */
	void reconfigureCallback(flipper_control::FlipperControlConfig&config, uint32_t level);

	ros::Publisher markerPublisher;
	ros::Publisher frontFlipperAngleDesiredPub;
	ros::Publisher  rearFlipperAngleDesiredPub;

	ros::Publisher planeRotationPub;

	ros::NodeHandle nodeHandle_;

	ros::Timer msg_timer ;

	std::string tf_prefix;
	//**************** tf frames
	std::string BASE_FRAME;
	std::string NEXT_BASE_FRAME;
	std::string ROTATED_NEXT_BASE_FRAME;
	std::string STATIC_NEXT_BASE_FRAME;
	std::string MAP_FRAME;
	std::string ODOM_FRAME;
	std::string FLIPPER_FRONT_LEFT_FRAME;
	std::string FLIPPER_FRONT_RIGHT_FRAME;
	std::string FLIPPER_REAR_LEFT_FRAME;
	std::string FLIPPER_REAR_RIGHT_FRAME;

	std::string FLIPPER_REGION_FRONT_LEFT_FRAME;
	std::string FLIPPER_REGION_FRONT_RIGHT_FRAME;
	std::string FLIPPER_REGION_REAR_LEFT_FRAME;
	std::string FLIPPER_REGION_REAR_RIGHT_FRAME;
	// Obeject declarations
	CropMap cropMap;
	FitPlane fitPlane;
	NESM nesm;
	FlipperAngles flipperAngles;

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
	double fitPlaneLength;
	double S_NE_thresshold;

};


#endif /* ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_FLIPPERCONTROL_H_ */
