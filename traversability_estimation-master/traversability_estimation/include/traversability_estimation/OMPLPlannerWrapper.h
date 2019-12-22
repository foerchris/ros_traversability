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

#include <robot_navigation/TrackingControl.h>


class OMPLPlanner
{

	public:
		OMPLPlanner ();
		~OMPLPlanner ();

		/*!
		 * plan a path from current pose to goal pose
		 * @param goal_d; goal pose
		 * @param waypoints; final path
		 */
		void plan (pose goal_d, std::vector<pose> &waypoints);

		/*!
		 * set reference to traversability map
		 * @param traversabilityMap; pointer to object of traversabilityMap
		 */
		void setTraversabilityMap (traversability_estimation::TraversabilityMap* traversabilityMap);

		/*!
		 * set reference to traversability map as gridmap
		 * @param travGridMap; traversability map as Grid Map
		 */
		void setTraversabilityGridMap(grid_map::GridMap travGridMap);

		/*!
		 * service call to calculate path
		 * @param request
		 * @param response
		 * @return
		 */
		bool clcPathCallback( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

		/*!
		 * Transform pose relative to robot to pose relative to map
		 * @param Pose; pose to transform
		 */
		void robotToGlobaleTransform(pose &Pose);

		/*!
		 * Transform pose relative to map to pose relative to robot
		 * @param Pose; pose to transform
		 */
		void globaleToRobotTransform(pose &Pose);



	private:

		std::vector<std::vector<int>> allowedAngle_map;
		grid_map::GridMap allowedMap;
		Eigen::Array2i mapSize;
		double resolution;
		double mapsizeX;
		double mapsizeY;

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
