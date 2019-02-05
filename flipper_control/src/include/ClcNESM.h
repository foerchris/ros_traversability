/*
 * ClcNESM.h
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
class ClcNESM
{
	public:
	ClcNESM();
	virtual ~ClcNESM();

	double clcNESMStabilityMeasure(const geometry_msgs::Pose& g1, const geometry_msgs::Pose& g2, const geometry_msgs::Pose& c);


	cv::Vec3d g1_prime_public;
	cv::Vec3d g2_prime_public;
	cv::Vec3d c_prime_public;
	cv::Vec3d g1_public;
	cv::Vec3d g2_public;
	cv::Vec3d c_public;
	cv::Vec3d p_highest_public;
	private:
	cv::Vec3d clcQuaternion(const cv::Vec3d& v,const tf2::Quaternion& q);
	double magnitude(const cv::Vec3d& v);

	double NESM_thresshold;


};

#endif /* ROS_ROBOCUP_FLIPPER_CONTROL_SRC_INCLUDE_CLCNESM_H_ */
