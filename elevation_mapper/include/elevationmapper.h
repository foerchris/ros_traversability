/*
 * elevationmapper.h
 *
 *  Created on: Mar 1, 2018
 *      Author: kfabian
 */

#ifndef SRC_ELEVATIONMAPPER_H_
#define SRC_ELEVATIONMAPPER_H_

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class ElevationMapper {
public:
	ElevationMapper(double size_x, double size_y, double resolution, std::string frame);
	virtual ~ElevationMapper();
	void addPointCloud(sensor_msgs::PointCloud2ConstPtr laser);
	void addPointCloud(sensor_msgs::PointCloud2Ptr laser);
	void addPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr laser);

	void resetMap(double size_x, double size_y, double resolution, std::string frame);
	grid_map_msgs::GridMapConstPtr getMapMessage()
	{
		grid_map_msgs::GridMap* message = new grid_map_msgs::GridMap;
		grid_map::GridMapRosConverter::toMessage(map, *message);
		return grid_map_msgs::GridMapConstPtr(message);
	}
	grid_map::GridMap getMap()
	{

		return map;
	}
private:
	void adaptMap(double x, double y);

	grid_map::GridMap map;

	double max_valid_t;
	double max_valid_b;
	double max_valid_l;
	double max_valid_r;
	double max_map_t = 0;
	double max_map_b = 0;
	double max_map_l = 0;
	double max_map_r = 0;
};

#endif /* SRC_ELEVATIONMAPPER_H_ */
