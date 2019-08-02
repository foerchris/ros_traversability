/*
 * YamlToParamterSrv.cpp
 *
 *  Created on: Aug 30, 2017
 *      Author: chfo
 */

#include "traversability_estimation/GetObjectInfoFromYaml.h"

GetObjectInfoFromYaml::GetObjectInfoFromYaml(ros::NodeHandle& nodeHandle)
			: nodeHandle_(nodeHandle)

{
	path = ros::package::getPath("traversability_estimation")+"/config";
	yamlLoaded = false;
}

GetObjectInfoFromYaml::~GetObjectInfoFromYaml()
{

}

void GetObjectInfoFromYaml::loadYaml(const std::string& configName)
{
	std::vector<boost::filesystem::path> filesRobotConfigs;
	try
	{
		objectConfig = YAML::LoadFile(path +"/"+ configName + ".yaml");

		yamlLoaded = true;

		int objectCount = objectConfig["drl_world"]["objects"].size();

		for (int j = 0; j < objectCount; j++)
		{
			std::cout<<"object"<< objectConfig["drl_world"]["objects"][j]["name"].as<std::string>()<<std::endl;
		}

	}
	catch(YAML::ParserException& e)
	{
		std::cout << "Load YAML Robot configuration file error: " << e.what () << "\n";
	}
}
int GetObjectInfoFromYaml::numPossibleObjects()
{
	if(yamlLoaded)
	{
		return objectConfig["drl_world"]["objects"].size();;
	}
	else
	{
		ROS_ERROR("Yaml file not loaded please execute GetObjectInfoFromYaml::loadYaml() first");
		return 0;
	}
}

int GetObjectInfoFromYaml::numThisObjects(const int& index)
{
	if(yamlLoaded)
	{
		return objectConfig["drl_world"]["objects"][index]["number"].as<int>();
	}
	else
	{
		ROS_ERROR("Yaml file not loaded please execute GetObjectInfoFromYaml::loadYaml() first");
		return 0;
	}
}


std::string GetObjectInfoFromYaml::getName(const int& index)
{
	//std::cout<<"index"<<index<<std::endl;
	if(yamlLoaded)
	{
		return objectConfig["drl_world"]["objects"][index]["name"].as<std::string>();
	}
	else
	{
		ROS_ERROR("Yaml file not loaded please execute GetObjectInfoFromYaml::loadYaml() first");
		return 0;
	}
}

void GetObjectInfoFromYaml::getinitPose(const int& index, object_options& objectOptions)
{

	if(yamlLoaded)
	{
		double min = 0, max = 0;
		objectOptions.mirrorObject = objectConfig["drl_world"]["objects"][index]["mirrored_object"].as<bool>();
		if(objectOptions.mirrorObject)
		{
			objectOptions.length = objectConfig["drl_world"]["objects"][index]["shift"].as<double>();
		}


		objectOptions.x.minMaxcase = returnValue(index,objectConfig["drl_world"]["objects"][index]["initPose"]["positionX"].as<std::string>(),min,max);

		objectOptions.x.min = min;
		objectOptions.x.max = max;

		objectOptions.y.minMaxcase = returnValue(index,objectConfig["drl_world"]["objects"][index]["initPose"]["positionY"].as<std::string>(),min,max);

		objectOptions.y.min = min;
		objectOptions.y.max = max;

		objectOptions.z.minMaxcase = returnValue(index,objectConfig["drl_world"]["objects"][index]["initPose"]["positionZ"].as<std::string>(),min,max);

		objectOptions.z.min = min;
		objectOptions.z.max = max;

		objectOptions.roll.minMaxcase = returnValue(index,objectConfig["drl_world"]["objects"][index]["initPose"]["orientationR"].as<std::string>(),min,max);

		objectOptions.roll.min = min;
		objectOptions.roll.max = max;

		objectOptions.pitch.minMaxcase = returnValue(index,objectConfig["drl_world"]["objects"][index]["initPose"]["orientationP"].as<std::string>(),min,max);

		objectOptions.pitch.min = min;
		objectOptions.pitch.max = max;

		objectOptions.yaw.minMaxcase = returnValue(index,objectConfig["drl_world"]["objects"][index]["initPose"]["orientationY"].as<std::string>(),min,max);

		objectOptions.yaw.min = min;
		objectOptions.yaw.max = max;

	}
	else
	{
		ROS_ERROR("Yaml file not loaded please execute GetObjectInfoFromYaml::loadYaml() first");
	}
}

int GetObjectInfoFromYaml::returnValue(const int& index,const std::string& object, double& min, double& max)
{

	std::size_t foundDots = object.find("..");
	std::size_t foundSlash = object.find("/");

	if(foundDots < 100)
	{

		std::istringstream iss (object.substr(0,  foundDots));
		iss >> min;

		std::istringstream iss2 (object.substr(foundDots+2,  object.size()));

		iss2 >> max;

		return 1;
	}
	else if(foundSlash < 100)
	{
		std::istringstream iss (object.substr(0,  foundSlash));
		iss >> min;

		std::istringstream iss2 (object.substr(foundSlash+1,  object.size()));
		iss2 >> max;

		return 2;
	}
	else
	{

		std::istringstream iss (object.substr(0,  object.size()));
		iss >> min;

		return 0;
	}
}

