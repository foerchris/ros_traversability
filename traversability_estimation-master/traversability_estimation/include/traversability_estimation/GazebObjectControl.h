/*
 * GazebObjectControl..h
 *
 *  Created on: 04.07.2019
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_TRAVERSABILITY_ESTIMATION_MASTER_TRAVERSABILITY_ESTIMATION_INCLUDE_TRAVERSABILITY_ESTIMATION_GAZEBOBJECTCONTROL_H_
#define ROS_ROBOCUP_TRAVERSABILITY_ESTIMATION_MASTER_TRAVERSABILITY_ESTIMATION_INCLUDE_TRAVERSABILITY_ESTIMATION_GAZEBOBJECTCONTROL_H_



#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

class GazebObjectControl
{

 public:
	GazebObjectControl (ros::NodeHandle& nodeHandle);
	~GazebObjectControl ();
	void spwanObject(const std::string& modelName, const std::string& xmlName, geometry_msgs::Pose startPose);
	void deleteObject(const std::string& modelName);
	void setObject(const std::string& modelName, geometry_msgs::Pose startPose);



 private:
	std::string  readXmlFile(const std::string& name);

	ros::ServiceClient setModelClient;
	ros::ServiceClient spawnModelClient;
	ros::ServiceClient deleteModelClient;

	//! ROS node handle.
	ros::NodeHandle& nodeHandle_;

	std::string modelPath;
	std::string gazeboMoveObjectFrame;
};


#endif /* ROS_ROBOCUP_TRAVERSABILITY_ESTIMATION_MASTER_TRAVERSABILITY_ESTIMATION_INCLUDE_TRAVERSABILITY_ESTIMATION_GAZEBOBJECTCONTROL_H_ */
