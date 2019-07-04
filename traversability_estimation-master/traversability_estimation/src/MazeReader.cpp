/*
 * get_text_file_data.cpp
 *
 *  Created on: 08.03.2019
 *      Author: chfo
 */

#include "traversability_estimation/MazeReader.h"


MazeReader::MazeReader ()
{
	//std::ofstream myfile;
	//path = ros::package::getPath("traversability_estimation")+"/mazegenerator-master";
	path = "/scratch-local/cdtemp/chfo/traverability/traversability_estimation/Gazebo Script";
	//path = "/home/chfo/Dropbox/Masterarbeit/python code/traversability_estimation/Gazebo Script";

	env_size = 10.0;
	possible_cells = getPosibleCells();
	forbidden_list.clear();
}

MazeReader::~MazeReader ()
{
}

std::vector<maze> MazeReader::readTextFile (const int& num)
{
	std::vector<maze> maze_vect;

	float xmax;
	float xresolution;

	float ymax;
	float yresolution;

	float xfactor;
	float yfactor;
	std::string line;
	std::ifstream myfile (path + "/maze" + std::to_string(num) + ".txt");
	if (myfile.is_open())
	{

		while ( getline (myfile,line) )
	    {
			std::string name;
			maze maze_value;
			std::stringstream ss;
			ss.str(line);
			ss >> name;
			if( name == "xmax")
			{
				ss >> xmax >> name >> ymax >> name >> ymax;
			}
			else if(name == "xresolution")
			{
				ss >> xresolution >> name >> yresolution;
				xfactor = xmax/xresolution;
				yfactor = ymax/yresolution;

			}
			else
			{
				float x1, x2, y1, y2;
				ss >> x1 >> name >> x2 >> name >> y1 >> name >> y2;


				maze_value.x = fabs(x1 + x2)/2 * xfactor /2.66667 * env_size;
				maze_value.y = fabs(y1 + y2)/2 * yfactor /2.66667 * env_size;

				if(fabs(x1 - x2) >= 0.0001)
				{
					maze_value.orientation = 90;
				}
				else
				{
					maze_value.orientation = 0;
				}

				if(maze_value.x<env_size-0.1 && maze_value.y<env_size-0.1 &&  maze_value.x>0.1 && maze_value.y>0.1)
				{
					maze_value.x = maze_value.x - env_size/2;
					maze_value.y = maze_value.y - env_size/2;
					maze_vect.push_back(maze_value);
				}

			}
	    }
	    myfile.close();
	}
	return maze_vect;
}

std::vector<maze> MazeReader::getPosibleCells ()
{
	std::vector<maze> values;

	for (float x = env_size/8; x <= env_size - env_size/8; x += env_size/4)
	{
		for (float y = env_size/8; y <= env_size - env_size/8; y += env_size/4)
		{
			maze value;
			value.x = x - env_size/2;
			value.y = y - env_size/2;
			values.push_back(value);
		}
	}
	return values;
}

maze MazeReader::getRandomCell()
{
	std::random_device rd;
	std::mt19937 mt(rd());
    std::uniform_int_distribution<> dis(0, possible_cells.size()-1);
    std::uniform_int_distribution<> dis_orientation(-180, 180);
    int rand_cell_number;

    for(std::size_t i = 0; i<10; i++)
    {
        rand_cell_number = dis(mt);
        if(!(std::find(forbidden_list.begin(), forbidden_list.end(), rand_cell_number) != forbidden_list.end()))
        {
        	break;
        }

    }

    forbidden_list.push_back(rand_cell_number);

    maze random_cell = possible_cells[rand_cell_number];
	random_cell.orientation = dis_orientation(mt);

	return random_cell;
}

void MazeReader::reset()
{
	forbidden_list.clear();
}
