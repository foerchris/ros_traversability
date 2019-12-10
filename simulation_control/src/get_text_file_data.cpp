/*
 * get_text_file_data.cpp
 *
 *  Created on: 08.03.2019
 *      Author: chfo
 */

#include "simulations_kontrolle/MazeReader.h"


int main(int argc, char** argv) {
	ros::init(argc, argv, "get_text_file_data");
	ros::NodeHandle nh;
	ros::Rate rate (10);

	MazeReader mazeReader;

	std::vector<maze> maze_vect;
	maze_vect = mazeReader.readTextFile(1);

	for( auto value : maze_vect)
	{
		std::cout<<"x="<<value.x<<"\ty="<<value.y<<"\ty1="<<value.orientation<<std::endl;
	}

	for(int i = 0; i<15; i++)
	{
		maze value = mazeReader.getRandomCell();
		std::cout<<"x="<<value.x<<"\ty="<<value.y<<"\ty1="<<value.orientation<<std::endl;
		if(i==9)
		{
			mazeReader.reset();
		}

	}


	ros::spinOnce ();
	rate.sleep();
	return 0;
}


