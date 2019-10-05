/*
 * YamlToMsgConverter.h
 *
 *  Created on: Aug 30, 2017
 *      Author: chfo
 */

#ifndef SRC_ROBOT_NAVIGATION_FOR_3D_STRUCTURES_SRC_YAMLTOMSGCONVERTER_H_
#define SRC_ROBOT_NAVIGATION_FOR_3D_STRUCTURES_SRC_YAMLTOMSGCONVERTER_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <string>


class YamlToMsgConverter {
	public:
		YamlToMsgConverter();
		~YamlToMsgConverter();
		void get_all(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret);
		std::string robotBehaviorsConfigFile;
		std::string robotBehaviorsConfigName;

	private:
		ros::NodeHandle nodeHandle();
};



#endif /* SRC_ROBOT_NAVIGATION_FOR_3D_STRUCTURES_SRC_YAMLTOMSGCONVERTER_H_ */
