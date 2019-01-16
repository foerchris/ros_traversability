/*
 * elevationmapper.cpp
 *
 *  Created on: Mar 1, 2018
 *      Author: kfabian
 */

#include "elevationmapper.h"

ElevationMapper::ElevationMapper(double size_x, double size_y, double resolution, std::string frame) {
	  // Create grid map.
	  map = grid_map::GridMap({"elevation"});
	  map.setFrameId(frame);
	  map.setGeometry(grid_map::Length(size_x, size_y), resolution);

	  grid_map::Position center = map.getPosition();
	  max_valid_t = center[0] + map.getLength().x() / 2;
	  max_valid_b = center[0] - map.getLength().x() / 2;
	  max_valid_l = center[1] - map.getLength().y() / 2;
	  max_valid_r = center[1] + map.getLength().y() / 2;

	  ROS_INFO("Created map with size %f x %f m (%i x %i cells).", map.getLength().x(), map.getLength().y(), map.getSize()(0), map.getSize()(1));
}
void ElevationMapper::addPointCloud(sensor_msgs::PointCloud2ConstPtr laser){
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempLaser (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*laser, *tempLaser);
	addPointCloud(tempLaser);
}
void ElevationMapper::addPointCloud(sensor_msgs::PointCloud2Ptr laser){
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempLaser (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::moveFromROSMsg(*laser, *tempLaser);
	addPointCloud(tempLaser);
}
void ElevationMapper::addPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr laser){

	//std::cout << "points: " << laser->points.size() << std::endl;
	for (int i = 0; i < laser->points.size(); i++)
	{
		//std::cout << "test  0  x:" << laser->points.at(i).x << " y:" << laser->points.at(i).y << std::endl;
		adaptMap(laser->points.at(i).x, laser->points.at(i).y);
		if(!map.isInside(grid_map::Position(laser->points.at(i).x, laser->points.at(i).y)))
			continue;
		//std::cout << "test  " << map.atPosition("elevation", grid_map::Position(laser->points.at(i).x, laser->points.at(i).y)) << std::endl;
		if (!(map.atPosition("elevation", grid_map::Position(laser->points.at(i).x, laser->points.at(i).y)) > laser->points.at(i).z))
		{
			//std::cout << "test 2 " << std::endl;
			map.atPosition("elevation", grid_map::Position(laser->points.at(i).x, laser->points.at(i).y)) = laser->points.at(i).z;
		}
	}
	map.setTimestamp(pcl_conversions::fromPCL(laser->header.stamp).toNSec());
}

void ElevationMapper::adaptMap(double x, double y)
{
	if(!map.isInside(grid_map::Position(x, y)))
	{
		double diff_t = 0;
		double diff_b = 0;
		double diff_l = 0;
		double diff_r = 0;
		bool resize = false;
		double r_t = 0;
		double r_b = 0;
		double r_l = 0;
		double r_r = 0;

		if (x > max_valid_t)
		{
			diff_t = x - max_valid_t;
		}
		if (x < max_valid_b)
		{
			diff_b = x - max_valid_b;
		}
		if (y > max_valid_r)
		{
			diff_r = y - max_valid_r;
		}
		if (y < max_valid_l)
		{
			diff_l = y - max_valid_l;
		}
		if (((-(max_valid_b - max_map_b) - diff_t) < 1) && (diff_t != 0))
		{
			r_t = x + 3;
			resize = true;
		}
		if (((max_valid_t - max_map_t) - (abs(diff_b)) < 1) && (diff_b != 0))
		{
			r_b = x - 3;
			resize = true;
		}
		if (((-(max_valid_l - max_map_l) - diff_r) < 1) && (diff_r != 0))
		{
			r_l = y + 3;
			resize = true;
		}
		if ((((max_valid_r - max_map_r) - abs(diff_l)) < 1) && (diff_l != 0))
		{
			r_r = y - 3;
			resize = true;
		}

		if (resize)
		{
			grid_map::GridMap temp({"elevation"});
			temp.setFrameId(map.getFrameId());
			temp.setGeometry(grid_map::Length(0.5, 0.5), map.getResolution());
			temp.setPosition(grid_map::Position(r_t + r_b, r_l + r_r));
			temp.atPosition("elevation", grid_map::Position(r_t + r_b, r_l + r_r)) = 1;
			ROS_INFO("before map with size %f x %f m (%i x %i cells).", map.getLength().x(), map.getLength().y(), map.getSize()(0), map.getSize()(1));
			map.extendToInclude(temp);
			ROS_INFO("after map with size %f x %f m (%i x %i cells).", map.getLength().x(), map.getLength().y(), map.getSize()(0), map.getSize()(1));		}
		else
		{
			map.move(grid_map::Position(map.getPosition().x() + diff_t + diff_b + ((diff_t != 0) * 0.7) - ((diff_b != 0) * 0.7),
					map.getPosition().y() + diff_l + diff_r + ((diff_r != 0) * 0.7) - ((diff_l != 0) * 0.7)));
		}
		grid_map::Position center = map.getPosition();
		max_valid_t = center[0] + map.getLength().x() / 2;
		max_valid_b = center[0] - map.getLength().x() / 2;
		max_valid_l = center[1] - map.getLength().y() / 2;
		max_valid_r = center[1] + map.getLength().y() / 2;
	}
	if (x > max_map_t)
		max_map_t = x;
	if (x < max_map_b)
		max_map_b = x;
	if (y < max_map_l)
		max_map_l = y;
	if (y > max_map_r)
		max_map_r = y;
}

void ElevationMapper::resetMap(double size_x, double size_y, double resolution, std::string frame)
{
	map.clear("elevation");
	// Create grid map.
	map = grid_map::GridMap({"elevation"});
	map.setFrameId(frame);
	map.setGeometry(grid_map::Length(size_x, size_y), resolution);

	grid_map::Position center = map.getPosition();
	max_valid_t = center[0] + map.getLength().x() / 2;
	max_valid_b = center[0] - map.getLength().x() / 2;
	max_valid_l = center[1] - map.getLength().y() / 2;
	max_valid_r = center[1] + map.getLength().y() / 2;

	ROS_INFO("Created map with size %f x %f m (%i x %i cells).", map.getLength().x(), map.getLength().y(), map.getSize()(0), map.getSize()(1));
}
ElevationMapper::~ElevationMapper() {
	// TODO Auto-generated destructor stub
}

