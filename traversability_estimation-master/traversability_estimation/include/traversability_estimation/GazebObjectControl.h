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

#include <std_srvs/Empty.h>

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>


#include <chrono>
#include <thread>

#include <flipper_control/GetContactPoints.h>
#include "traversability_estimation/GetObjectInfoFromYaml.h"


class GazebObjectControl
{

 public:
	GazebObjectControl (ros::NodeHandle& nodeHandle);
	~GazebObjectControl ();
	void spwanObject(const std::string& modelName, const std::string& xmlName, geometry_msgs::Pose startPose);
	void deleteObject(const std::string& modelName);
	void setObject(const std::string& modelName, geometry_msgs::Pose startPose);
	//void publischGoal(const geometry_msgs::Pose& goalPose);
	void destroyWorld();
	void generateWorld(int minObjects, int maxObjects);
	void setRobotZeroPose();
	void clcGoalPathSrvsCall();
	void creatEnviroment();



 private:
	std::string  readXmlFile(const std::string& name);
	void poseToOdomMsg(const geometry_msgs::Pose& pose, nav_msgs::Odometry& setPose);
	visualization_msgs::Marker createMarker (std::string ns, int id, double x, double y,  double r, double g , double b , double a);
	bool resetRobotSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	geometry_msgs::Pose tfTransform(const geometry_msgs::Pose& pose,const std::string& destination_frame,const std::string& original_frame);
	geometry_msgs::Pose setRandomObst(const object_options& objectOptions, const bool& mirror, const double& lastX);
	double creatRndPosition(const min_max_object_pose& minMaxObjectPose);
	void publischGoal(const ros::TimerEvent& event);
	void resetCallback(const ros::TimerEvent& event);
	void generateWorld2();
	void resetAllObjects();
	void setObjectInWorld();

	void MapImageCallback(const sensor_msgs::ImageConstPtr& msg);


	ros::ServiceClient setModelClient;
	ros::ServiceClient spawnModelClient;
	ros::ServiceClient deleteModelClient;
	ros::ServiceClient clcPathClient;
	ros::ServiceServer resetRobot;

	ros::Publisher elevationMapImagePublisher;
	ros::Publisher goalPosePublischer;
	ros::Publisher markerPublisher;

	//! ROS node handle.
	ros::NodeHandle& nodeHandle_;
	ros::Timer msg_timer;
	ros::Timer reset_timer;

	std::string modelPath;
	std::string gazeboMoveObjectFrame;
	std::vector<std::string> objects;
	struct objectNameIndex {
	  std::string name;
	  std::size_t yamlIndex;
	} ;
	std::vector<objectNameIndex> spwanedObjects;

	cv_bridge::CvImagePtr cv_ptr;
	//*****
	double resultion;
	double mapSizeX;
	double mapSizeY;

	bool resetWorld;
	bool calculating;

	geometry_msgs::Pose goalPose;

	cv::Mat globalMapImage;
	bool mapImageSet;

	std::string tf_prefix;
	std::string BASE_FRAME;
	std::string MAP_FRAME;
	std::string ODOM_FRAME;

	std::unique_ptr<tf::TransformListener> tfListener;

	visualization_msgs::MarkerArray markerArray;

	GetContactPoints getContactPoints;
	GetObjectInfoFromYaml getObjectInfoFromYaml_;

};


#endif /* ROS_ROBOCUP_TRAVERSABILITY_ESTIMATION_MASTER_TRAVERSABILITY_ESTIMATION_INCLUDE_TRAVERSABILITY_ESTIMATION_GAZEBOBJECTCONTROL_H_ */
