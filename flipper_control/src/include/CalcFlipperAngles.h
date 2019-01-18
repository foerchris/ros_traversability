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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>


class CalcFlipperAngles
{
	public:
	CalcFlipperAngles();
	virtual ~CalcFlipperAngles();
	std::vector<geometry_msgs::Pose> clcNewPoses(const std::vector<geometry_msgs::Pose>& poses,tf2::Quaternion q);

	private:
	void tfBoradcaster(const geometry_msgs::Pose poses);

	static tf2_ros::StaticTransformBroadcaster static_broadcaster;



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
