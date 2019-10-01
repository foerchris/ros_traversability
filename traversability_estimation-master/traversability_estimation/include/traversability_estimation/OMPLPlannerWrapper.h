/*
 * OMPLPlannerWrapper.h
 *
 *  Created on: Nov 11, 2017
 *      Author: Andreas Nofen
 */


#include <ompl/base/State.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>


 
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

 
#include <opencv2/opencv.hpp>

#include <grid_map_ros/grid_map_ros.hpp>

#include <ros/package.h>

// TF transfrom
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include "traversability_estimation/TraversabilityMap.hpp"

struct pose {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
} ;

class OMPLPlanner
{
	private:
		std::vector<std::vector<int>> allowedAngle_map;
		grid_map::GridMap allowedMap;
		bool isStateValid (const ompl::base::State *state);
		Eigen::Array2i mapSize;
		double resolution;
		double mapsizeX;
		double mapsizeY;
	public:
		OMPLPlanner ();
		~OMPLPlanner ();
		void plan (pose goal_d, std::vector<pose> &waypoints);
		void setTraversabilityMap (traversability_estimation::TraversabilityMap* traversabilityMap);
		void setTraversabilityGridMap(grid_map::GridMap travGridMap);
		bool clcPathCallback( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		// Transfrom pose relative to robot to pose relative to map
		void robotToGlobaleTransform(pose &Pose);
		//ompl::base::OptimizationObjectivePtr getTraversabilityObjectivconst(ompl::base::SpaceInformationPtr& si);

		// Transfrom pose relative to map to pose relative to robot
		void globaleToRobotTransform(pose &Pose);
		std::string tf_prefix;
		std::vector<std::vector<int>> hahdh;

		traversability_estimation::TraversabilityMap* traversabilityMap_;

		std::unique_ptr<tf::TransformListener> tfListener;
		tf::StampedTransform transform;
		cv::Mat trasverability_map_0;
		cv::Mat trasverability_map_45;
		cv::Mat trasverability_map_90;
		std::string BASE_FRAME;
		std::string ODOM_FRAME;
		std::string MAP_FRAME;
};
