/*
 * SnakeMode.h
 *
 *  Created on: 20.05.2018
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_SNAKEMODE_H_
#define ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_SNAKEMODE_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <ctime>

struct VecXZPhi
{
	double x;
	double z;
	double phi;
};
struct DistanceVec
{
	std::vector<double> x;		// distance
	std::vector<double> y;		// distance2
	std::vector<double> z;		// height
	double vd;		// vertical distance from body to ground, measured by USS
};


struct VecRPY
{
		double roll;
		double pitch;
		double yaw;
};

struct BarVec
{
		std::vector<double> center_x;
		std::vector<double> center_y;
		std::vector<double> center_z;
		std::vector<double> width;
};
class SnakeMode
{
	public:
		SnakeMode ();
		virtual ~SnakeMode ();
		void snakeMode (double &frontDesiredAngle,double &rearDesiredAngle);
		void snakeModeVec (DistanceVec distanceVec, VecXZPhi &desiredVec);
		void snakeModeVecAbs(DistanceVec distanceVec, VecXZPhi &desiredVec);
		double snakeModeAngle (double z);
		void SetSnakeModeParams (double flipperAngleMax, double flipperAngleMin);
		void DistVect (DistanceVec frontDist, DistanceVec rearDist);
		void SetBodyAngle (double pitch, double roll, double angle);
		void SetUss (double front, double mid1, double mid2, double rear);
	private:
		double R;		// Radius of the wheel of the robot body
		double r;		// Radius of the wheel on the end of the flipper
		double l;		// Wheel distance between the robot wheel axis and the wheel axis on the end of the flipper
		double d;		// Distance between body wheel axis and "downside of flipper wheel"
		double theta;	// Angle of distance (see above) and flipper wheel radius

		double flipper_angle_min;
		double flipper_angle_max;
		double front_uss;
		double mid1_uss;
		double mid2_uss;
		double rear_uss;
		DistanceVec front;
		DistanceVec rear;
		ros::Timer timer;
		double currentTime;
		double previousTime;

};


#endif /* ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_SNAKEMODE_H_ */
