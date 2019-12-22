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

	/*!
	 * caclulate hyperplane
	 * @param poses points from croped elevation map
	 * @return hyperplane
	 */
	FittedPlane fitPlane(const std::vector<geometry_msgs::Pose>& poses);

	/*!
	 * return rotation from hyperplane
	 * @param fittedPlane; hyperplane
	 * @return rotation as quaternion
	 */
	tf2::Quaternion getRotations(FittedPlane fittedPlane);

	std::vector<geometry_msgs::Pose> samplePlane(const FittedPlane& fittedPlane,const double& xLength,const double& yLength,const double& resulution);

	std::vector<geometry_msgs::Pose> sampleLine(const double& angle ,const double& xLength,const double& resulution);
	private:
	/*!
	 * calculate mean from poses
	 * @param poses
	 * @return mean pose
	 */
	geometry_msgs::Pose clcMean(const std::vector<geometry_msgs::Pose>& poses);

	/*!
	 * calculate cross mean from poses
	 * @param poses
	 * @return cross mean pose
	 */
	std::vector<double> clcCrossMean(const std::vector<geometry_msgs::Pose>& poses);



};

#endif /* ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_FITPLANE_H_ */
