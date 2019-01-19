/*
 * CalcFlipperAngles.h
 *
 *  Created on: 16.01.2019
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_CALCFLIPPERANGLES_H_
#define ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_CALCFLIPPERANGLES_H_


// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// Tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
/*
struct minMaxFlipperVel {
	double min;
	double max;
	cv::Point min_loc;
	cv::Point max_loc;
};*/

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

class CalcFlipperAngles
{
	public:
	CalcFlipperAngles();
	virtual ~CalcFlipperAngles();
	std::vector<geometry_msgs::Pose> clcNewPoses(const std::vector<geometry_msgs::Pose>& poses,tf2::Quaternion q);
	flipperContactPointsAngles clcContactAngles(const std::vector<geometry_msgs::Pose>& values, std::string flipperFrame);
	//void clcFlipperAngles(const std::vector<minMaxFlipperVel>& minMaxVel, flipperAngles& robotFlipperAngles);
	geometry_msgs::Pose tfTransform(const geometry_msgs::Pose& pose,const std::string& destination_frame,const std::string& original_frame);

	private:
	std::unique_ptr<tf::TransformListener> tfListener;
	tf::StampedTransform transform;


	// Robot parameter
	double R;		// Radius of the wheel of the robot body
	double r;		// Radius of the wheel on the end of the flipper
	double L;		// Wheel distance between the robot wheel axis and the wheel axis on the end of the flipper
	//double d;		// Distance between body wheel axis and "downside of flipper wheel"
	double theta;	// Angle of distance (see above) and flipper wheel radius
	double dThreshold;
	double trackLength;

	// *********** the image variables
	double xLength  ;
	double yLength  ;
	double TracksBaseLinkDist;
	double FlipperTrackLength;


	// **** parameter for valide pose
	double velocitiy_robot;
	double delta_t;

};


#endif /* ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_CALCFLIPPERANGLES_H_ */
