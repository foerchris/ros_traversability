/*
 * YamlToMsgConverter.cpp
 *
 *  Created on: Aug 30, 2017
 *      Author: chfo
 */

#include "YamlToMsgConverter.h"

const std::string PARAM_ROBOT_CONFIG_PATH = "/robot_configs";

YamlToMsgConverter::YamlToMsgConverter()
{
	std::string robotConfigsPath=ros::package::getPath("robot_navigation")+PARAM_ROBOT_CONFIG_PATH;
	std::vector<boost::filesystem::path> filesRobotConfigs;
	get_all(robotConfigsPath,".yaml",filesRobotConfigs);

	if (filesRobotConfigs.size() < 1)
	{
		ROS_INFO("No Robot Configuration file found");
		return;
	}
	for (std::size_t i = 0; i < filesRobotConfigs.size(); i++)
	{
		try
		{
			std::size_t found; // Example: /GETbot/start_camera_movement Start Camera

			// Find end of topic and split it
			found = filesRobotConfigs[i].generic_string().find_first_of(".");
			robotBehaviorsConfigName = filesRobotConfigs[i].generic_string().substr(0,found);	//	/GETbot/start_camera_movement
			robotBehaviorsConfigFile.clear();

			std::string fullConfigPath=robotConfigsPath+"/"+filesRobotConfigs[i].generic_string();

			YAML::Node robotConfig = YAML::LoadFile(fullConfigPath);

			std::ifstream myReadFile;
			myReadFile.open(fullConfigPath);
			while(myReadFile) // To get you all the lines.
			{
				std::string addstring;
				getline(myReadFile,addstring); // Saves the line in STRING.
				robotBehaviorsConfigFile=robotBehaviorsConfigFile+"\n"+addstring;
			}
			myReadFile.close();

		}
		catch(YAML::ParserException& e)
		{
			std::cout << "Load YAML Robot configuration file error: " << e.what () << "\n";
		}
	}
}

//https://stackoverflow.com/questions/11140483/how-to-get-list-of-files-with-a-specific-extension-in-a-given-folder
void YamlToMsgConverter::get_all(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret)
{
    if(!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return;

    boost::filesystem::recursive_directory_iterator it(root);
    boost::filesystem::recursive_directory_iterator endit;

    while(it != endit)
    {
        if(boost::filesystem::is_regular_file(*it) && it->path().extension() == ext) ret.push_back(it->path().filename());
        ++it;

    }

}
YamlToMsgConverter::~YamlToMsgConverter()
{

}


