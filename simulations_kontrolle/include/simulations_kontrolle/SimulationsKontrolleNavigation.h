/*
 * SimulationsKontrolleNavigation..h
 *
 *  Created on: 04.07.2019
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_simulations_kontrolle_MASTER_simulations_kontrolle_INCLUDE_simulations_kontrolle_SimulationsKontrolleNavigation_H_
#define ROS_ROBOCUP_simulations_kontrolle_MASTER_simulations_kontrolle_INCLUDE_simulations_kontrolle_SimulationsKontrolleNavigation_H_



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
#include "simulations_kontrolle/MazeReader.h"


class SimulationsKontrolleNavigation
{

 public:
	SimulationsKontrolleNavigation (ros::NodeHandle& nodeHandle);
	~SimulationsKontrolleNavigation ();




 private:

	// listener for reseting scenario when episode ends
	void creatEnviroment();
	// timer callback for creatEnviroment
	void resetCallback(const ros::TimerEvent& event);

	// spawn an object in gazebo by service call
	void spwanObject(const std::string& modelName, const std::string& xmlName, geometry_msgs::Pose startPose);
	// delete an object in gazebo by service call
	void deleteObject(const std::string& modelName);
	// set existing object to pose in gazebo by service call
	void setObject(const std::string& modelName, geometry_msgs::Pose startPose);
	//void publischGoal(const geometry_msgs::Pose& goalPose);

	// delete all spawned objects in gazebo created by the node
	void destroyWorld();
	// spawn and set all objects according to yaml file
	void generateWorld();

	// set all objects to random pose
	void setObjectInWorld(const bool& setMaze);
	// set all objects to reset pose
	void resetAllObjects();
	// set objects to random pose
	geometry_msgs::Pose setRandomObst(const object_options& objectOptions, const bool& mirror, const double& lastX);
	// create random pose for objects
	double creatRndPosition(const min_max_object_pose& minMaxObjectPose);
	// create random orientation for objects
	geometry_msgs::Pose creatRandomOrientation(geometry_msgs::Pose pose);

	// set the robot to reset pose
	void setRobotZeroPose();
	// set the robot to zero pose service server
	bool resetRobotSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	// set the robots to randome start pose
	void setRobotStartPose(geometry_msgs::Pose startPose);

	// for goal point visualization
	visualization_msgs::Marker createMarker (std::string ns, int id, double x, double y,  double r, double g , double b , double a);
	// helper method for reading xml file to get robot description
	std::string  readXmlFile(const std::string& name);
	// helper method pose to odom msg
	void poseToOdomMsg(const geometry_msgs::Pose& pose, nav_msgs::Odometry& setPose);
	// helper method maze to pose type
	geometry_msgs::Pose transformMaze(maze position);
	// timer for publishing goal position relative to robot coordinates
	void publischGoal(const ros::TimerEvent& event);
	// helper method  tf transformation
	geometry_msgs::Pose tfTransform(const geometry_msgs::Pose& pose,const std::string& destination_frame,const std::string& original_frame);

	// callback subscribe elevation_map_image
	void MapImageCallback(const sensor_msgs::ImageConstPtr& msg);
	// callback subscribe odom
	void odomCallback (const nav_msgs::OdometryConstPtr& odomMsg);







	ros::ServiceClient setModelClient;
	ros::ServiceClient spawnModelClient;
	ros::ServiceClient deleteModelClient;
	ros::ServiceClient clcPathClient;
	ros::ServiceServer resetRobot;

	ros::Publisher elevationMapImagePublisher;
	ros::Publisher goalPosePublischer;
	ros::Publisher currentPosePublischer;
	ros::Publisher markerPublisher;

	ros::Subscriber odomSub;

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
	std::vector<objectNameIndex> mazeObjectList;
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
	int robot_number = 1;

	std::string tf_prefix;
	std::string BASE_FRAME;
	std::string MAP_FRAME;
	std::string ODOM_FRAME;

	std::unique_ptr<tf::TransformListener> tfListener;

	visualization_msgs::MarkerArray markerArray;

	CropMap cropMap;
	GetObjectInfoFromYaml getObjectInfoFromYaml_;
	MazeReader mazeReader;


};


#endif /* ROS_ROBOCUP_simulations_kontrolle_MASTER_simulations_kontrolle_INCLUDE_simulations_kontrolle_SimulationsKontrolleNavigation_H_ */
