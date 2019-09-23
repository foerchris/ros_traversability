/*
 * RobotPathPlanner.h
 *
 *  Created on: 29.07.2018
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_ROBOT_PATH_PLANNING_SRC_INCLUDE_ROBOTPATHPLANNER_H_
#define ROS_ROBOCUP_ROBOT_PATH_PLANNING_SRC_INCLUDE_ROBOTPATHPLANNER_H_
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMapInfo.h>
#include <grid_map_msgs/GetGridMap.h>
#include "traversability_estimation/OMPLPlannerWrapper.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>

class RobotPathPlanner
{

 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
	RobotPathPlanner(ros::NodeHandle& nodeHandle, traversability_estimation::TraversabilityMap& pathPlanningtraversabilityMap);
  /*!
   * Destructor.
   */
  virtual ~RobotPathPlanner();

  void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& gridMsg);
 private:
  bool clcPath(const grid_map::GridMap& gridMsg, std::vector<pose>& Pose);
  void goalPoseCallback(const nav_msgs::Odometry::ConstPtr& goalPoseMsg);
  bool clcPathCallback( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  visualization_msgs::Marker createMarker (std::string ns, int id, double x, double y,  double r, double g, double b, double a);
  ros::Subscriber traverabilitySubscriber;
  ros::Subscriber GoalSubscriber;
  pose goalPose;
  traversability_estimation::TraversabilityMap& traversabilityMap_;
  ros::ServiceServer clcPathSrv;
  bool clcThePath;
  bool goalSet;
  ros::NodeHandle& nodeHandle_;
  ros::Publisher markerPublisher;
  std::string tf_prefix;
	double speed;

};



#endif /* ROS_ROBOCUP_ROBOT_PATH_PLANNING_SRC_INCLUDE_ROBOTPATHPLANNER_H_ */
