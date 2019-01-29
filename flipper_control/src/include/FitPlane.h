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

struct FittedPlane {
	double a;
	double b;
	double c;
};

class FitPlane
{
	public:
	FitPlane();
	virtual ~FitPlane();
	std::vector<geometry_msgs::Pose> samplePlane(const FittedPlane& fittedPlane,const double& xLength,const double& yLength,const double& resulution);

	FittedPlane fitPlane(const std::vector<geometry_msgs::Pose>& poses);
	tf2::Quaternion getRotations(FittedPlane fittedPlane);
	private:
	geometry_msgs::Pose clcMean(const std::vector<geometry_msgs::Pose>& poses);
	std::vector<double> clcCrossMean(const std::vector<geometry_msgs::Pose>& poses);

};

#endif /* ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_FITPLANE_H_ */
