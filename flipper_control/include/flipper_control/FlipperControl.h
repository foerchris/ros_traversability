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

	void MapImageCallback(const sensor_msgs::ImageConstPtr& msg);

	void FlipperSequenzCallback(const ros::TimerEvent& event);

	void SequenceControl(cv::Mat mapImage);

	tf2::Quaternion groundPlane(cv::Mat image, double* maxZValue, FittedPlane* fittedPlane);

	tf2::Quaternion newPlane(const cv::Mat& image, double* maxZValue, FittedPlane* fittedPlane, const double& setRoll, const double& setPitch);


	MaxFlipperContactPointsAngles flipperRegion(cv::Mat image,const tf2::Quaternion& quat, const double& maxZ, const std::string& flipperFrame, const std::string& flipperRegionFrame);

	///*********************** stability analysis for all possibility's
	double stabilityAnalysis(const geometry_msgs::Pose& g1,const geometry_msgs::Pose& g2, const geometry_msgs::Pose& c, const std::string& frame1, const std::string& frame2,  const bool& rotatePitch, const int& rotationDirection);
	int cheakNESM(MaxFlipperContactPointsAngles frontLeft, MaxFlipperContactPointsAngles frontRight, MaxFlipperContactPointsAngles rearLeft, MaxFlipperContactPointsAngles rearRight);
	void displayAllRotatedPoints(const std::string& position, const std::string& frame);
	void displayRotatedPoints(const cv::Vec3d& point, const std::string& name, const std::string& frame, float r, float g, float b);


	double returnBiggerVel(const double& vel1, const double& vel2);

	void publishAngles (RobotFlipperAngles robotFlipperAngles);

	geometry_msgs::Pose addPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);
	geometry_msgs::Pose subPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);

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
