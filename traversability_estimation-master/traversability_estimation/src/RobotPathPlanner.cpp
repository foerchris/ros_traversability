/*
 * RobotPathPlanner.cpp
 *
 *  Created on: 29.07.2018
 *      Author: chfo
 */

#include "traversability_estimation/RobotPathPlanner.h"

#include <opencv2/opencv.hpp>


RobotPathPlanner::RobotPathPlanner(ros::NodeHandle& nodeHandle, traversability_estimation::TraversabilityMap& pathPlanningtraversabilityMap)
	: nodeHandle_(nodeHandle),
	  traversabilityMap_(pathPlanningtraversabilityMap)
{
	traverabilitySubscriber = nodeHandle_.subscribe<grid_map_msgs::GridMap>("traversability_estimation/traversability_map",1,boost::bind (&RobotPathPlanner::gridMapCallback, this, _1));
	clcPathSrv = nodeHandle_.advertiseService("clcPath", &RobotPathPlanner::clcPathCallback, this);

	GoalSubscriber = nodeHandle_.subscribe<nav_msgs::Odometry>("goal_pose",1,boost::bind(&RobotPathPlanner::goalPoseCallback, this, _1));
	markerPublisher = nodeHandle_ .advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 20);
	tf_prefix = ros::this_node::getNamespace();
	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);
	goalSet = false;
	clcThePath = false;
	goalPose.x = 0;
	goalPose.y = 0;
	goalPose.z = 0;
	goalPose.roll = 0;
	goalPose.pitch = 0;
	goalPose.yaw = 0;
}

RobotPathPlanner::~RobotPathPlanner()
{

}
visualization_msgs::Marker RobotPathPlanner::createMarker (std::string ns, int id, double x, double y,  double r, double g, double b, double a)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = tf_prefix+"/map";
	marker.header.stamp = ros::Time();
	marker.ns = ns;

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(0);

	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a; // Don't forget to set the alpha!

    marker.id = id;

	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	return marker;
}

void RobotPathPlanner::gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& gridMsg)
{
	grid_map::GridMap traverabilityMap;
	grid_map::GridMapRosConverter::fromMessage(*gridMsg,traverabilityMap);
	std::vector<pose> wayPoints;
	if(clcPath(traverabilityMap,wayPoints))
	{

	}
}

bool RobotPathPlanner::clcPathCallback( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if(goalSet)
	{
		std::vector<pose> poses;
		//omplPlanner.plan(goalPose,poses);

		//printf ("poses size: %i\n", poses.size());

		visualization_msgs::MarkerArray markerArray;

		for(std::size_t i=0; i<poses.size();i++)
		{
			printf ("pose x: %4.4f pose y: %4.4f \n", poses.at(i).x, poses.at(i).y);
			markerArray.markers.push_back (createMarker("static line", i, poses.at(i).x, poses.at(i).y, 1.0, 1.0, 0.0, 1.0));
		}
		markerPublisher.publish(markerArray);
	}
	return true;
}


void RobotPathPlanner::goalPoseCallback(const nav_msgs::Odometry::ConstPtr& goalPoseMsg)
{
	goalSet=true;
	goalPose.x =goalPoseMsg->pose.pose.position.x;
	goalPose.y =goalPoseMsg->pose.pose.position.y;
	goalPose.yaw = tf::getYaw(goalPoseMsg->pose.pose.orientation);

}
bool RobotPathPlanner::clcPath(const grid_map::GridMap& travGridMap, std::vector<pose>& Pose)
{
	//OMPLPlanner omplPlanner(traversabilityMap_);
	//omplPlanner.setTraversabilityGridMap(travGridMap);


	return true;
}
