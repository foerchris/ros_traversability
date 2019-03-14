/*
 * MazeReader.h
 *
 *  Created on: 08.03.2019
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_TRAVERSABILITY_ESTIMATION_MASTER_TRAVERSABILITY_ESTIMATION_INCLUDE_MAZEREADER_H_
#define ROS_ROBOCUP_TRAVERSABILITY_ESTIMATION_MASTER_TRAVERSABILITY_ESTIMATION_INCLUDE_MAZEREADER_H_

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

struct maze{
	float x;
	float y;
	float orientation;
};

class MazeReader
{

 public:
	MazeReader ();
	~MazeReader ();
	std::vector<maze> readTextFile (const int& num);
	maze getRandomCell();
	void reset();


 private:
	std::vector<maze> getPosibleCells ();

	std::vector<maze> possible_cells;
	std::vector<int> forbidden_list;
	std::string path;
	float env_size;


};



#endif /* ROS_ROBOCUP_TRAVERSABILITY_ESTIMATION_MASTER_TRAVERSABILITY_ESTIMATION_INCLUDE_MAZEREADER_H_ */
