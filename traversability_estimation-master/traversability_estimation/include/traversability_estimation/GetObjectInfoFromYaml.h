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
		void loadYaml(const std::string& configName);
		int numPossibleObjects();
		std::string getName(const int& index);
		void getinitPose(const int& index, object_options& objectOptions);


	private:
		int returnValue(const int& index,const std::string& object, double& min, double& max);


		std::string path;
		ros::NodeHandle& nodeHandle_;

		YAML::Node objectConfig;
		bool yamlLoaded;
};

#endif /* SRC_ROBOT_BEHAVIORS_FOR_3D_STRUCTURES_SRC_getObjectInfoFromYaml_H_ */
