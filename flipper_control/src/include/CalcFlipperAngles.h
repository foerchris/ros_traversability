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


class CalcFlipperAngles
{
	public:
	CalcFlipperAngles();
	virtual ~CalcFlipperAngles();

	private:
	std::vector<geometry_msgs::Pose> isTrackInRange(const std::vector<geometry_msgs::Pose>& poses);
	geometry_msgs::Pose clcDesiredPose(const std::vector<geometry_msgs::Pose>& poses);
	geometry_msgs::Pose clcMean(const std::vector<geometry_msgs::Pose>& poses);
	std::vector<double> clcCrossMean(const std::vector<geometry_msgs::Pose>& poses);


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
