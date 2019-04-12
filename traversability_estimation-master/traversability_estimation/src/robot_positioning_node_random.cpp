/*
 * robot_positioning.cpp
 *
 *  Created on: 10.10.2018
 *      Author: chfo
 */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetModelState.h>

#include <tf2/LinearMath/Quaternion.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_srvs/Empty.h>

#include <chrono>
#include <thread>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sstream>

#include <random>
#include <iostream>

#include "traversability_estimation/MazeReader.h"


ros::Publisher goalPosePublischer;
ros::Publisher markerPublisher;

visualization_msgs::Marker createMarker (std::string ns, int id, double x, double y,  double r, double g , double b , double a);
void creatRandomPose(geometry_msgs::Pose& pose, int xyInterval);
void poseToOdomMsg(const geometry_msgs::Pose& pose, nav_msgs::Odometry& setPose);
void setObjectPose(const std::string& name, geometry_msgs::Pose startPose, gazebo_msgs::SetModelState& setmodelstate, bool robot);
geometry_msgs::Pose mapToBaseLinkTransform(geometry_msgs::Pose pose);
geometry_msgs::Pose mapToOdomTransform(geometry_msgs::Pose pose);
void setGETjagZeroPose(const std::string& name, gazebo_msgs::SetModelState& setmodelstate);
geometry_msgs::Pose setRandomObst();
geometry_msgs::Pose transformMaze(maze position);
std::string BASE_FRAME = "/base_link";
std::string MAP_FRAME = "/map";
std::string ODOM_FRAME = "/odom";
std::string gazeboMoveObjectFrame = "world";
std::string tf_prefix = "//GETjag1";
std::unique_ptr<tf::TransformListener> tfListener;
tf::StampedTransform transform;
MazeReader mazeReader;

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_positioning");
	ros::NodeHandle nh;

	tf_prefix = ros::this_node::getNamespace();
	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	std::istringstream iss (tf_prefix.substr(6, tf_prefix.size()));
	int robot_number = 1;
	iss >> robot_number;

	BASE_FRAME = tf_prefix + BASE_FRAME;
	MAP_FRAME = tf_prefix + MAP_FRAME;
	ODOM_FRAME = tf_prefix + ODOM_FRAME;
	ros::Rate rate (10);
	ros::Rate rate2sec (0.5);

	tfListener = std::unique_ptr<tf::TransformListener> (new tf::TransformListener);

	ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

	goalPosePublischer = nh.advertise<nav_msgs::Odometry>("goal_pose", 20);
	markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 20);


	gazebo_msgs::SetModelState setmodelstate;
	geometry_msgs::Pose startPose;
	bool reset = true;
	std::string resetParam = "End_of_episode";
	geometry_msgs::Pose mapGoalPose;
	visualization_msgs::MarkerArray markerArray;


	std::vector<maze> maze_vect;

	while(ros::ok())
	{
		if(reset == true)
		{
			mazeReader.reset();
			nh.setParam(resetParam,false);
			reset = false;
			// set all getjag to a zero pose
			setGETjagZeroPose(tf_prefix, setmodelstate);
			if (client.call(setmodelstate))
			{
				//ROS_INFO("Successful to call service: Set GETjag!!!");
			}
			else
			{
				ROS_ERROR("Failed to call service: Set GETjag!!!");
				return 1;
			}


			// reset obstacles
			startPose.position.x = 15;
			startPose.position.y = 15;
			startPose.position.z = 0;
			for(int i=1; i<=10+1; i++)
			{
				int objectIndex;
				if(tf_prefix == "GETjag1")
				{
					objectIndex=i;
				}
				else if(tf_prefix == "GETjag2")
				{
					objectIndex=i+15;
				}
				else if(tf_prefix == "GETjag3")
				{
					objectIndex=i+30;
				}
				else if(tf_prefix == "GETjag4")
				{
					objectIndex=i+45;
				}
				else if(tf_prefix == "GETjag5")
				{
					objectIndex=i+60;
				}
				else if(tf_prefix == "GETjag6")
				{
					objectIndex=i+75;
				}

				// set obstacles
				setObjectPose("object_robocup_wall250_clone_"+std::to_string(objectIndex), startPose,setmodelstate, false);
				if (client.call(setmodelstate))
				{
					//ROS_INFO("Successful to call service: Set Obstacle: %i",i);
				}
				else
				{
					ROS_ERROR("Failed to call service: Set Obstacle: %i",i);
					return 1;
				}
			}

			std::this_thread::sleep_for(std::chrono::seconds(1));
			maze_vect = mazeReader.readTextFile(robot_number);



			int possibleRandomObstacles = 4;
			int minObstacles = 3;
			std::random_device rd;
			std::mt19937 mt(rd());
			std::uniform_real_distribution<double> obst(minObstacles, possibleRandomObstacles);

			int anzObstacles = obst(mt);
			for(int i=1; i<=anzObstacles+1; i++)
			{
				creatRandomPose(startPose, 5);
				//startPose = transformMaze(mazeReader.getRandomCell());
				startPose = mapToOdomTransform(startPose);

				int objectIndex;
				if(tf_prefix == "GETjag1")
				{
					objectIndex=i;
				}
				else if(tf_prefix == "GETjag2")
				{
					objectIndex=i+15;
				}
				else if(tf_prefix == "GETjag3")
				{
					objectIndex=i+30;
				}
				else if(tf_prefix == "GETjag4")
				{
					objectIndex=i+45;
				}
				else if(tf_prefix == "GETjag5")
				{
					objectIndex=i+60;
				}
				else if(tf_prefix == "GETjag6")
				{
					objectIndex=i+75;
				}

				// set obstacles
				setObjectPose("object_robocup_wall250_clone_"+std::to_string(objectIndex), startPose,setmodelstate, false);
				if (client.call(setmodelstate))
				{
					//ROS_INFO("Successful to call service: Set Obstacle: %i",i);
				}
				else
				{
					ROS_ERROR("Failed to call service: Set Obstacle: %i",i);
					return 1;
				}
			}
/*

			for(int i=1; i<=possibleRandomObstacles+1; i++)
			{
				startPose = setRandomObst();
				startPose = mapToOdomTransform(startPose);

				int objectIndex;
				if(tf_prefix == "GETjag1")
				{
					objectIndex=i+10;
					std::cout<<"objectIndex="<<objectIndex<<std::endl;
				}
				else if(tf_prefix == "GETjag2")
				{
					objectIndex=i+25;
				}
				else if(tf_prefix == "GETjag3")
				{
					objectIndex=i+40;
				}
				else if(tf_prefix == "GETjag4")
				{
					objectIndex=i+55;
				}
				else if(tf_prefix == "GETjag5")
				{
					objectIndex=i+70;
				}
				else if(tf_prefix == "GETjag6")
				{
					objectIndex=i+85;
				}

				// set obstacles
				setObjectPose("object_robocup_wall250_clone_"+std::to_string(objectIndex), startPose,setmodelstate, false);
				if (client.call(setmodelstate))
				{
					//ROS_INFO("Successful to call service: Set Obstacle: %i",i);
				}
				else
				{
					ROS_ERROR("Failed to call service: Set Obstacle: %i",i);
					return 1;
				}
			}
*/
			// set getjag to a random pose
			creatRandomPose(startPose, 4.5);
			//startPose = transformMaze(mazeReader.getRandomCell());
			startPose = mapToOdomTransform(startPose);

			setObjectPose(tf_prefix, startPose, setmodelstate, true);
			if (client.call(setmodelstate))
			{
				//ROS_INFO("Successful to call service: Set GETjag!!!");
			}
			else
			{
				ROS_ERROR("Failed to call service: Set GETjag!!!");
				return 1;
			}

			std::this_thread::sleep_for(std::chrono::seconds(2));

			nh.setParam("reset_elevation_map",true);
			std::this_thread::sleep_for(std::chrono::seconds(5));

			// Create random goal position message
			//creatRandomPose(mapGoalPose, 4.5);
			mapGoalPose = transformMaze(mazeReader.getRandomCell());

			markerArray.markers.push_back (createMarker(tf_prefix+" Goal Pose", 1, mapGoalPose.position.x, mapGoalPose.position.y, 1.0, 1.0, 0.0,1.0));
			nh.setParam("Ready_to_Start_DRL_Agent",true);

		}

		if(nh.hasParam(resetParam))
		{
			nh.getParam(resetParam,reset);
		}

		geometry_msgs::Pose robotGoalPose;

		nav_msgs::Odometry goalPoseMsg;
		robotGoalPose = mapToBaseLinkTransform(mapGoalPose);
		poseToOdomMsg(robotGoalPose,goalPoseMsg);
		markerPublisher.publish(markerArray);
		goalPosePublischer.publish(goalPoseMsg);

		ros::spinOnce ();
		rate.sleep();
	}

	return 0;
}


geometry_msgs::Pose setRandomObst()
{
	geometry_msgs::Pose pose;
	maze randomCell = mazeReader.getRandomCell();
	pose = transformMaze(randomCell);

	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<double> roll_orientaton(M_PI/2, M_PI/2 + M_PI/4);
	std::uniform_real_distribution<double> height(0, 0.3);

	pose.position.z = height(mt);
	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0,  roll_orientaton(mt), randomCell.orientation/180*M_PI);

	pose.orientation.x = myQuaternion.x();
	pose.orientation.y = myQuaternion.y();
	pose.orientation.z = myQuaternion.z();
	pose.orientation.w = myQuaternion.w();
	std::cout<<"x="<<pose.position.x<<"y"<<pose.position.y<<std::endl;

	return pose;
}


geometry_msgs::Pose transformMaze(maze position)
{
	geometry_msgs::Pose pose;

	pose.position.x = position.x;

	pose.position.y = position.y;

	pose.position.z = 0;

	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0, 0, position.orientation/180*M_PI);

	pose.orientation.x = myQuaternion.x();
	pose.orientation.y = myQuaternion.y();
	pose.orientation.z = myQuaternion.z();
	pose.orientation.w = myQuaternion.w();
	return pose;
}

void creatRandomPose(geometry_msgs::Pose& pose, int xyInterval)
{
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<double> position(-xyInterval, xyInterval);


	pose.position.x = position(mt);

	pose.position.y = position(mt);

	pose.position.z = 0;

	std::uniform_real_distribution<double> orientaton(-M_PI, M_PI);

	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0, 0, orientaton(mt));

	pose.orientation.x = myQuaternion.x();
	pose.orientation.y = myQuaternion.y();
	pose.orientation.z = myQuaternion.z();
	pose.orientation.w = myQuaternion.w();
}

void poseToOdomMsg(const geometry_msgs::Pose& pose, nav_msgs::Odometry& setPose)
{
	 setPose.pose.pose.position.x = pose.position.x;
	 setPose.pose.pose.position.y = pose.position.y;
	 setPose.pose.pose.position.z = pose.position.z;

	 setPose.pose.pose.orientation.x =  pose.orientation.x;
	 setPose.pose.pose.orientation.y =  pose.orientation.y;
	 setPose.pose.pose.orientation.z =  pose.orientation.z;
	 setPose.pose.pose.orientation.w =  pose.orientation.w;
}

void setGETjagZeroPose(const std::string& name, gazebo_msgs::SetModelState& setmodelstate)
{
	gazebo_msgs::ModelState modelstate;
	modelstate.model_name = name;
	modelstate.reference_frame = gazeboMoveObjectFrame;

	geometry_msgs::Twist model_twist;
	model_twist.linear.x = 0;
	model_twist.linear.y = 0;
	model_twist.linear.z = 0;
	model_twist.angular.x = 0;
	model_twist.angular.y = 0.0;
	model_twist.angular.z = 0.0;
	modelstate.twist = model_twist;

	geometry_msgs::Pose startPose;
	if(name=="GETjag1")
	{
		startPose.position.x = 11.5;
		startPose.position.y = 11.5;
	}
	else if(name=="GETjag2")
	{
		startPose.position.x = 11.5;
		startPose.position.y = -11.5;

	}
	else if(name=="GETjag3")
	{
		startPose.position.x = -11.5;
		startPose.position.y = 11.5;

	}
	else if(name=="GETjag4")
	{
		startPose.position.x = -11.5;
		startPose.position.y = -11.5;

	}
	else if(name=="GETjag5")
	{
		startPose.position.x = 11.5;
		startPose.position.y = 21.5;

	}
	else if(name=="GETjag6")
	{
		startPose.position.x = -11.5;
		startPose.position.y = 21.5;
	}
	startPose.position.z = 1;



	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY( 0, 0, M_PI);

	startPose.orientation.x = myQuaternion.x();
	startPose.orientation.y = myQuaternion.y();
	startPose.orientation.z = myQuaternion.z();
	startPose.orientation.w = myQuaternion.w();

	modelstate.pose = startPose;

	setmodelstate.request.model_state = modelstate;
}


void setObjectPose(const std::string& name, geometry_msgs::Pose startPose, gazebo_msgs::SetModelState& setmodelstate, bool robot)
{
	gazebo_msgs::ModelState modelstate;
	modelstate.model_name = name;
	modelstate.reference_frame = gazeboMoveObjectFrame;

	geometry_msgs::Twist model_twist;
	model_twist.linear.x = 0;
	model_twist.linear.y = 0;
	model_twist.linear.z = 0;
	model_twist.angular.x = 0;
	model_twist.angular.y = 0.0;
	model_twist.angular.z = 0.0;
	modelstate.twist = model_twist;

	if(robot)
	{
		startPose.position.z = 2;
	}

	//std::cout<<name<<": Pose x: "<<startPose.position.x<< "Pose y: "<<startPose.position.y<<std::endl;
	modelstate.pose = startPose;

	setmodelstate.request.model_state = modelstate;
}

visualization_msgs::Marker createMarker (std::string ns, int id, double x, double y,  double r = 1.0, double g = 0.0, double b = 0.0, double a = 1.0)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = MAP_FRAME;
	marker.header.stamp = ros::Time();
	marker.ns = ns;

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(0);

	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a; // Don't forget to set the alpha!

    marker.id = id;

	marker.pose.position.x = x;
	marker.pose.position.y = y;
//printf("marker.pose.position.x %lf, marker.pose.position.y %lf: \n",marker.pose.position.x, marker.pose.position.y);

	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	return marker;
}

geometry_msgs::Pose mapToBaseLinkTransform(geometry_msgs::Pose pose)
{

	// TF transformation of the Point which is nearest to the robot
	const ros::Time& scanTimeStamp = ros::Time (0);
	try
	{
		tfListener->waitForTransform (BASE_FRAME, MAP_FRAME, scanTimeStamp, ros::Duration (3.0));
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}


	tf::Quaternion quat;
	quat.setX(pose.orientation.x);
	quat.setY(pose.orientation.y);
	quat.setZ(pose.orientation.z);
	quat.setW(pose.orientation.w);


	tf::Vector3 origin;

	origin.setX(pose.position.x);
	origin.setY(pose.position.y);
	origin.setZ(pose.position.z);

	tf::StampedTransform transform;

	tf::Stamped<tf::Pose> transPose (tf::Pose( quat,origin), scanTimeStamp, MAP_FRAME);

	tf::Stamped<tf::Pose> poseTransformed;

	try
	{
		tfListener->transformPose(BASE_FRAME, scanTimeStamp, transPose, MAP_FRAME, poseTransformed);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR(" Point xyz %s", ex.what ());
	}
	origin=poseTransformed.getOrigin();
	quat=poseTransformed.getRotation();

	geometry_msgs::Pose returnPose;
	returnPose.position.x=origin.getX();
	returnPose.position.y=origin.getY();
	returnPose.position.z=origin.getZ();
	returnPose.orientation.x = quat.x();
	returnPose.orientation.y = quat.y();
	returnPose.orientation.z = quat.z();
	returnPose.orientation.w = quat.w();



	return returnPose;
}


geometry_msgs::Pose mapToOdomTransform(geometry_msgs::Pose pose)
{

	//printf("befor: pose.position.x: %lf, pose.position.y: %lf, pose.position.z: %lf \n",pose.position.x, pose.position.y, pose.position.z);
	// TF transformation of the Point which is nearest to the robot
	const ros::Time& scanTimeStamp = ros::Time (0);
	try
	{
		tfListener->waitForTransform (ODOM_FRAME, MAP_FRAME, scanTimeStamp, ros::Duration (3.0));
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}


	tf::Quaternion quat;
	quat.setX(pose.orientation.x);
	quat.setY(pose.orientation.y);
	quat.setZ(pose.orientation.z);
	quat.setW(pose.orientation.w);


	tf::Vector3 origin;

	origin.setX(pose.position.x);
	origin.setY(pose.position.y);
	origin.setZ(pose.position.z);

	tf::StampedTransform transform;

	tf::Stamped<tf::Pose> transPose (tf::Pose( quat,origin), scanTimeStamp, MAP_FRAME);

	tf::Stamped<tf::Pose> poseTransformed;

	try
	{
		tfListener->transformPose(ODOM_FRAME, scanTimeStamp, transPose, MAP_FRAME, poseTransformed);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR(" Point xyz %s", ex.what ());
	}
	origin=poseTransformed.getOrigin();
	quat=poseTransformed.getRotation();

	geometry_msgs::Pose returnPose;
	returnPose.position.x=origin.getX();
	returnPose.position.y=origin.getY();
	returnPose.position.z=origin.getZ();
	returnPose.orientation.x = quat.x();
	returnPose.orientation.y = quat.y();
	returnPose.orientation.z = quat.z();
	returnPose.orientation.w = quat.w();


	//printf("after: returnPose.position.x: %lf, returnPose.position.y: %lf, returnPose.position.z: %lf \n",returnPose.position.x, returnPose.position.y, returnPose.position.z);

	return returnPose;
}
