/*
 * MazeReader.h
 *
 *  Created on: 08.03.2019
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_simulations_kontrolle_MASTER_simulations_kontrolle_INCLUDE_MAZEREADER_H_
#define ROS_ROBOCUP_simulations_kontrolle_MASTER_simulations_kontrolle_INCLUDE_MAZEREADER_H_

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
	/**
	* read maze and get wall positions
	* @param num; number of training environment
	* @return return position of maze walls
	*/
	std::vector<maze> readTextFile (const int& num);
	/**
	* get random cells form possible cells
	* @param num; number of objects
	* @return return random cell from possible cells
	*/
	maze getRandomCell();

	/**
	* reset the maze
	*/
	void reset();


 private:
	/**
	* get all possible cells
	* @return all possible centers of cells
	*/
	std::vector<maze> getPosibleCells ();


	std::vector<maze> possible_cells;
	std::vector<int> forbidden_list;
	std::string path;
	float env_size;


};



#endif /* ROS_ROBOCUP_simulations_kontrolle_MASTER_simulations_kontrolle_INCLUDE_MAZEREADER_H_ */
