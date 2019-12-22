/*
 * SimulationsKontrolleFlipper..h
 *
 *  Created on: 04.07.2019
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_simulations_kontrolle_MASTER_simulations_kontrolle_INCLUDE_simulations_kontrolle_SimulationsKontrolleFlipper_H_
#define ROS_ROBOCUP_simulations_kontrolle_MASTER_simulations_kontrolle_INCLUDE_simulations_kontrolle_SimulationsKontrolleFlipper_H_



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

#include <flipper_control/CropMap.h>
#include "simulations_kontrolle/GetObjectInfoFromYaml.h"


class SimulationsKontrolleFlipper
{

 public:
	SimulationsKontrolleFlipper (ros::NodeHandle& nodeHandle);
	~SimulationsKontrolleFlipper ();





 private:

	/**
	* listener for reseting scenario when episode ends
	*/
	void creatEnviroment();

	/**
	* timer callback for creatEnviroment
	* @param event; timer event for calling reset callback
	*/
	void resetCallback(const ros::TimerEvent& event);

	/**
	* spawn an object in gazebo by service call
	* @param modelName; name of the object to load
	* @param modelName; name of the xml file of the model
	* @param startPose; pose to set the object
	*/
	void spwanObject(const std::string& modelName, const std::string& xmlName, geometry_msgs::Pose startPose);

	/**
	* delete an object in gazebo by service call
	* @param modelName; name of the object to load
	*/
	void deleteObject(const std::string& modelName);

	/**
	* set existing object to pose in gazebo by service call
	* @param modelName; name of the object to load
	* @param startPose; pose to set the object
	*/
	void setObject(const std::string& modelName, geometry_msgs::Pose startPose);

	/**
	* delete all spawned objects in gazebo created by the node
	*/
	void destroyWorld();

	/**
	* spawn and set all objects according to yaml file
	*/
	void generateWorld2();

	/**
	* set all objects to random pose
	*/
	void setObjectInWorld();

	/**
	* set all objects to reset pose
	*/
	void resetAllObjects();

	/**
	* get pose for setting object random
	* @param objectOptions; option from yaml file to random set objects
	* @param mirror; option to set mirrored object
	* @param lastX; last x value to place mirrored object
	* @return random pose
	*/
	geometry_msgs::Pose setRandomObst(const object_options& objectOptions, const bool& mirror, const double& lastX);

	/**
	* create random pose for objects
	* @param min_max_object_pose; range of random value
	* @return random value
	*/
	double creatRndPosition(const min_max_object_pose& minMaxObjectPose);


	/**
	* set the robot to zero pose
	*/
	void setRobotZeroPose();

	/**
	*  set the robot to zero pose service server
	*/
	bool resetRobotSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	/**
	*  call service to call path by robot_navigation
	*/
	void clcGoalPathSrvsCall();

	/**
	*  for goal point visualization
	* @param ns; namespace
	* @param id; index of points
	* @param x,y; position of points
	* @param r,g,b,a; color of markers
	* @return marker array of points
	*/
	visualization_msgs::Marker createMarker (std::string ns, int id, double x, double y,  double r, double g , double b , double a);


	/**
	*  helper method for reading xml file to get robot description
	* @param name; name of the xml file
	* @return data from xml file as string
	*/
	std::string readXmlFile(const std::string& name);

	/**
	*  helper method pose to odom msg
	* @param pose; geometry_msg::Pose type
	* @param setPose; nav_msgs::Odometry type
	*/
	void poseToOdomMsg(const geometry_msgs::Pose& pose, nav_msgs::Odometry& setPose);

	/**
	*  helper method  tf transformation
	* @param pose; pose at origin frame
	* @param destination_frame; destination frame string
	* @param original_frame; origin frame string
	* @return pose at destination frame
	*/
	geometry_msgs::Pose tfTransform(const geometry_msgs::Pose& pose,const std::string& destination_frame,const std::string& original_frame);

	/**
	*  timer for publishing goal position relative to robot coordinates
	* @param event; timer event for publishing goal pose
	*/
	void publischGoal(const ros::TimerEvent& event);

	/**
	*  callback subscribe elevation_map_image
	*/
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

	CropMap cropMap;
	GetObjectInfoFromYaml getObjectInfoFromYaml_;

};


#endif /* ROS_ROBOCUP_simulations_kontrolle_MASTER_simulations_kontrolle_INCLUDE_simulations_kontrolle_SimulationsKontrolleFlipper_H_ */
