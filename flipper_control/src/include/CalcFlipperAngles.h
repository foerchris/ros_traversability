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
#include <opencv2/imgproc.hpp>

struct flipperContactPointsAngles {
	std::vector<geometry_msgs::Pose> pose;
	std::vector<double> phi1;
	std::vector<double> phi2;
	std::vector<double> phiContact;
	std::vector<double> flipperAngle;

};

struct maxflipperContactPointsAngles {
	geometry_msgs::Pose pose;
	double maxFlipperAngle;
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
	maxflipperContactPointsAngles clcContactAngles(const std::vector<geometry_msgs::Pose>& values);

	void setParameter(double p1, double p2, double p3, double p4);


	private:

	maxflipperContactPointsAngles maxFlipperAngle(const flipperContactPointsAngles& flipperAngles);

	double dThreshold;
	double R;
	double L;
	double r;



};


#endif /* ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_CALCFLIPPERANGLES_H_ */
