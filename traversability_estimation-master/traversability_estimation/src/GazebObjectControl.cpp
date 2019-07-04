/*
 * GazebObjectControl.cpp
 *
 *  Created on: 04.07.2019
 *      Author: chfo
 */

#include "traversability_estimation/GazebObjectControl.h"

using namespace std;

GazebObjectControl::GazebObjectControl(ros::NodeHandle& nodeHandle)
		: nodeHandle_(nodeHandle)
{
	//ofstream myfile;
	//path = ros::package::getPath("traversability_estimation")+"/mazegenerator-master";
	//path = "/scratch-local/cdtemp/chfo/traverability/traversability_estimation/Gazebo Script";
	//path = "/home/chfo/Dropbox/Masterarbeit/python code/traversability_estimation/Gazebo Script";
	modelPath = "/eda/gazebo/models/";

	setModelClient = nodeHandle_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	spawnModelClient = nodeHandle_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
	deleteModelClient = nodeHandle_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

	gazeboMoveObjectFrame = 'world';
}

GazebObjectControl::~GazebObjectControl ()
{
}



void GazebObjectControl::setObject(const string& modelName, geometry_msgs::Pose startPose)
{
	gazebo_msgs::SetModelState setmodelstate;
	gazebo_msgs::ModelState modelstate;
	modelstate.model_name = modelName;
	modelstate.reference_frame = "";

	geometry_msgs::Twist model_twist;
	model_twist.linear.x = 0;
	model_twist.linear.y = 0;
	model_twist.linear.z = 0;
	model_twist.angular.x = 0;
	model_twist.angular.y = 0.0;
	model_twist.angular.z = 0.0;
	modelstate.twist = model_twist;

	modelstate.pose = startPose;

	setmodelstate.request.model_state = modelstate;

	if (setModelClient.call(setmodelstate))
	{
		ROS_INFO("Successful to call service: Set Model: %s",modelName.c_str());
	}
	else
	{
		ROS_ERROR("Successful to call service: Set Model: %s",modelName.c_str());
		//return 1;
	}
}

void GazebObjectControl::spwanObject(const string& modelName, const string& xmlName, geometry_msgs::Pose startPose)
{
	gazebo_msgs::SpawnModel spawnModel;

	spawnModel.request.model_name = modelName;

	string xmlFile = readXmlFile(xmlName);
	spawnModel.request.model_xml = xmlFile;
	spawnModel.request.robot_namespace = "";
	spawnModel.request.initial_pose = startPose;
	spawnModel.request.reference_frame = "";

	if (spawnModelClient.call(spawnModel))
	{
		ROS_INFO("Successful to call service: Spawn Model: %s",modelName.c_str());
	}
	else
	{
		ROS_ERROR("Successful to call service: Spawn Model: %s",modelName.c_str());
		//return 1;
	}
}

void GazebObjectControl::deleteObject(const string& modelName)
{
	gazebo_msgs::DeleteModel deleteModel;

	deleteModel.request.model_name = modelName;


	if (deleteModelClient.call(deleteModel))
	{
		ROS_INFO("Successful to call service: Delete Model: %s",modelName.c_str());
	}
	else
	{
		ROS_ERROR("Successful to call service: Delete Model: %s",modelName.c_str());
		//return 1;
	}
}

string  GazebObjectControl::readXmlFile(const string& name)
{
	string filePath = modelPath + name + "/model.sdf";

	ifstream t(filePath);
	string xmlText((std::istreambuf_iterator<char>(t)),
	                 std::istreambuf_iterator<char>());
	return xmlText;
}

