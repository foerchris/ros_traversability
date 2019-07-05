/*
 * approachPoint.h
 *
 *  Created on: Mar 28, 2018
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_APPROACHPOINT_H_
#define ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_APPROACHPOINT_H_
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <std_srvs/Empty.h>

#include "ContourPathPlanner.h"

class RobotDrive {

	public:
		RobotDrive (ros::NodeHandle& nodeHandle);
		virtual ~RobotDrive ();
	private:
		ros::NodeHandle& nodeHandle_;
		void getGoalPose(const nav_msgs::Odometry::ConstPtr& goalPoseMsg);
		geometry_msgs::Pose tfTransform(const geometry_msgs::Pose& pose,const std::string& destination_frame,const std::string& original_frame);
		bool clcPathSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		void clcPath(double planningRadius, std::vector<pose> Poses);
		std::vector<pose> pathPoses;

		ros::Subscriber goalSub;

	    ros::ServiceServer selectRegion;
	    ros::ServiceServer resetRobot;
		std::unique_ptr<tf::TransformListener> tfListener;
		std::string tf_prefix;
		std::string BASE_FRAME;
		std::string MAP_FRAME;
		std::string ODOM_FRAME;

		bool drive;
		ContourPathPlanner contourPathPlanner;

};


#endif /* ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_APPROACHPOINT_H_ */
