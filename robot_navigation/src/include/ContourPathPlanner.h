/*
 * TrackingControl.h
 *
 *  Created on: Oct 5, 2017
 *      Author: chfo
 */

#ifndef SRC_ROBOT_NAVIGATION_FOR_3D_STRUCTURES_SRC_ContourPathPlanner_H_
#define SRC_ROBOT_NAVIGATION_FOR_3D_STRUCTURES_SRC_ContourPathPlanner_H_

#include <cmath>
#include <iostream>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "TrackingControl.h"


enum chosenFollowAlg
{
	SimpleYDeviationControl, SimpleThetaDeviationControl, ReferencePathControl
};
// this class provides methods to calculate velocities depending on a goal pos and publish this velocities
class ContourPathPlanner {
public:
	ContourPathPlanner();
	virtual ~ContourPathPlanner ();

	bool followPath(chosenFollowAlg followAlg,bool resetPath);
	void forwardBackwarMode(const bool& forwardView,const bool& forwardMode, int direction);
	void setPathPoints(std::vector<pose> inputPoints);
	void clcPathToPoints(double planRadius);
	void clcPathToSinglePoint(double planRadius);
	void startRobotMovementfromCont(bool startMov);

private:
	visualization_msgs::Marker createMarker (std::string ns, int id, double x, double y,  double r, double g , double b , double a);
	void plan(const ompl::base::StateSpacePtr& space, bool easy);
	void printTrajectory(const ompl::base::StateSpacePtr& space,  const std::vector<double>& pt);
	void printPolyTrajectory(const ompl::base::StateSpacePtr& space,  const std::vector<pose>& pt);
	void printCircleTrajectory(const ompl::base::StateSpacePtr& space,  const std::vector<double>& pt);
	std::vector<pose> Poses;

	std::vector<pose> startPoses;

	ros::Publisher markerPublisher;

	ros::NodeHandle thisNodeHandel;

	bool pathForwardView;
	bool pathReversOriantation;

	std::string tf_prefix;
	std::string BASE_FRAME;
	std::string MAP_FRAME;
	std::string ODOM_FRAME;

	TrackingControl* trackingControl = new TrackingControl(thisNodeHandel, 0.5, 0.2, 0.05);



};


#endif /* SRC_ROBOT_NAVIGATION_FOR_3D_STRUCTURES_SRC_Pathplaner_H_ */
