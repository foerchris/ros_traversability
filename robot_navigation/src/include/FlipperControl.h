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
	std::vector<geometry_msgs::Pose> getContactPoints(cv::Mat flipperMaps, std::string flipperFrame);

	void displayFlipperPoints(const std::vector<geometry_msgs::Pose>& pose, const std::string& name,const std::string& frame);
	void DrawRotatedRectangle(cv::Mat& image,cv::RotatedRect rotatedRectangle);

	void FlipperSequenzCallback(const ros::TimerEvent& event);
	void publishAngles (flipperAngles robotFlipperAngles);


	void SequenceControl(cv::Mat mapImage);
	std::vector<cv::Mat> getFlipperRegions(cv::Mat mapImage);
	cv::Mat getCropedImage(geometry_msgs::Pose& pose, cv::Mat mapImage);

	std::vector<cv::Mat> getTrackedRegions(cv::Mat mapImage, const std::string& flipperFrame);
	cv::Mat getTracksImage(geometry_msgs::Pose& pose, cv::Mat mapImage);

	double clcFlipperAngles(flipperContactPointsAngles flipperLeft, flipperContactPointsAngles flipperRight);
	visualization_msgs::Marker createMarker (std::string ns, int id, double x, double y,  double r, double g, double b, double a);

	geometry_msgs::Pose FlipperControl::clcDesiredPose(const geometry_msgs::Pose& meanPose);

	geometry_msgs::Pose clcMean(std::vector<geometry_msgs::Pose> poses);

	std::vector<geometry_msgs::Pose> procTrackMaps(cv::Mat flipperMaps, const int& flipperLeftRight, std::string flipperFrame);

	std::string BASE_FRAME;
	std::string ODOM_FRAME;
	std::string MAP_FRAME;
	ros::NodeHandle nodeHandle_;

	std::unique_ptr<tf::TransformListener> tfListener;
	tf::StampedTransform transform;

	std::string tf_prefix;

	ros::Publisher frontFlipperAngleDesiredPub;
	ros::Publisher rearFlipperAngleDesiredPub;
	ros::Publisher markerPublisher;


	cv::Mat globalMapImage;
	bool mapImageSet;
	ros::Timer msg_timer ;
	// Robot parameter
	double R;		// Radius of the wheel of the robot body
	double r;		// Radius of the wheel on the end of the flipper
	double L;		// Wheel distance between the robot wheel axis and the wheel axis on the end of the flipper
	//double d;		// Distance between body wheel axis and "downside of flipper wheel"
	double theta;	// Angle of distance (see above) and flipper wheel radius
	double dThreshold;
	double trackLength;
	/////*********************** the image variables
	double resultion;
	double xLength  ;
	double yLength  ;
	double FlipperTrackLength;
	double mapSizeX;
	double mapSizeY;
	double TracksBaseLinkDist;
};


#endif /* ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_FLIPPERCONTROL_H_ */
