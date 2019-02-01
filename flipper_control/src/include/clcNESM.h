/*
 * clcNESM.h
 *
 *  Created on: 01.02.2019
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_CLCNESM_H_
#define ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_CLCNESM_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/core.hpp>
class clcNESM
{
	public:
	clcNESM();
	virtual ~clcNESM();

	double clcNESMStabilityMeasure(const geometry_msgs::Pose& g1, const geometry_msgs::Pose& g2, const geometry_msgs::Pose& c);

	private:
	cv::Vec3d clcQuaternion(const cv::Vec3d& v,const tf2::Quaternion& q);
	double magnitude(const cv::Vec3d& v);

	double NESM_thresshold;

};

#endif /* ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_CLCNESM_H_ */
