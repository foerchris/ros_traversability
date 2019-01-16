/*
 * object_follower_server.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: chfo
 */
#include <RobotControl.h>
#include "YamlToMsgConverter.h"


const std::string PARAM_ROBOT_CONFIG_PATH = "/Untitled Folder";

void loadRobotConfigToParamSever()
{
	ros::NodeHandle nh;
	YamlToMsgConverter yamlToMsgConverter;
	nh.setParam("Gui/"+yamlToMsgConverter.robotBehaviorsConfigName+"/RobotNavigation",yamlToMsgConverter.robotBehaviorsConfigFile);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_navigation_server");
	loadRobotConfigToParamSever();
	RobotControl robotControl;
	ros::spin();
	return 0;
}



