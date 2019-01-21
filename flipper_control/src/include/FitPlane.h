/*
 * FitPlane.h
 *
 *  Created on: 17.01.2019
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_FITPLANE_H_
#define ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_FITPLANE_H_

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

class FitPlane
{
	public:
	FitPlane();
	virtual ~FitPlane();
	tf2::Quaternion fitPlane(const std::vector<geometry_msgs::Pose>& poses);
	std::vector<geometry_msgs::Pose> isInRobotRange(const std::vector<geometry_msgs::Pose>& poses, const double& velocitiy_robot, const double& delta_t);
	std::vector<geometry_msgs::Pose> isInFlipperRange(const std::vector<geometry_msgs::Pose>& poses, const double& velocitiy_robot, const double& delta_t);

	private:
	// returns a,b,c fitted plan (z = a*x + b*y + c)
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
	double flipperLength;
	double TracksBaseLinkDist;
	double FlipperTrackLength;


	// **** parameter for valide pose
};

#endif /* ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_FITPLANE_H_ */
