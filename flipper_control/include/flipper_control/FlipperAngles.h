/*
 * FlipperAngles.h
 *
 *  Created on: 16.01.2019
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_FlipperAngles_H_
#define ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_FlipperAngles_H_


// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// Tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <opencv2/imgproc.hpp>

struct FlipperContactPointsAngles {
	std::vector<geometry_msgs::Pose> pose;
	std::vector<double> phi1;
	std::vector<double> phi2;
	std::vector<double> phiContact;
	std::vector<double> flipperAngle;

};

struct MaxFlipperContactPointsAngles {
	geometry_msgs::Pose pose;
	double maxFlipperAngle;
};


struct RobotFlipperAngles {
	double flipperAngleFront;
	double flipperAngleRear;
};

class FlipperAngles
{
	public:
	FlipperAngles();
	virtual ~FlipperAngles();
	MaxFlipperContactPointsAngles clcContactAngles(const std::vector<geometry_msgs::Pose>& values);

	void setParameter(double p1, double p2, double p3, double p4);


	private:

	MaxFlipperContactPointsAngles maxFlipperAngle(const FlipperContactPointsAngles& flipperAngles);

	double dThreshold;
	double R;
	double L;
	double r;



};


#endif /* ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_FlipperAngles_H_ */
