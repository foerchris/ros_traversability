/*
 * YamlToParamterSrv.h
 *
 *  Created on: Aug 30, 2017
 *      Author: chfo
 */

#ifndef SRC_ROBOT_BEHAVIORS_FOR_3D_STRUCTURES_SRC_getObjectInfoFromYaml_H_
#define SRC_ROBOT_BEHAVIORS_FOR_3D_STRUCTURES_SRC_getObjectInfoFromYaml_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <geometry_msgs/Pose.h>

struct min_max_object_pose {

	int minMaxcase;
	double min;
	double max;
} ;
struct object_options {

	bool mirrorObject;
	double length;
	min_max_object_pose x;
	min_max_object_pose y;
	min_max_object_pose z;
	min_max_object_pose roll;
	min_max_object_pose pitch;
	min_max_object_pose yaw;

} ;
class GetObjectInfoFromYaml {
	public:
		GetObjectInfoFromYaml(ros::NodeHandle& nodeHandle);
		~GetObjectInfoFromYaml();

		/**
		* load yaml file from specific location
		* @param configName;
		*/
		void loadYaml(const std::string& configName);

		/**
		* return number of objects define by the yaml file
		* @return number of objects
		*/
		int numPossibleObjects();

		/**
		* return name of the object from yaml file at index
		* @param index; index of the object
		* @return name of object
		*/
		std::string getName(const int& index);

		/**
		* return type of the object from yaml file at index
		* @param index; index of the object
		* @return type of object
		*/
		std::string getType(const int& index);

		/**
		* get allows poses for the object when placing random
		* @param index; index of the object
		* @param objectOptions; returns the the options to place objects
		*/
		void getinitPose(const int& index, object_options& objectOptions);

		/**
		* return number of object at index define by the yaml file
		* @param index; index of the object
		* @return random objects
		*/
		int numThisObjects(const int& index);


	private:

		/**
		* get values from opjectsOptions
		* @param index; index of the object
		* @param object; nanme of the object
		* @param min; min position
		* @param max; max position
		* @return return value
		*/
		int returnValue(const int& index,const std::string& object, double& min, double& max);

		std::string path;
		ros::NodeHandle& nodeHandle_;

		YAML::Node objectConfig;
		bool yamlLoaded;
};

#endif /* SRC_ROBOT_BEHAVIORS_FOR_3D_STRUCTURES_SRC_getObjectInfoFromYaml_H_ */
