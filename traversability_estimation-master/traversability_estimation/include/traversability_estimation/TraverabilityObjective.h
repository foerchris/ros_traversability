/*
 * OMPLPlannerWrapper.h
 *
 *  Created on: Nov 11, 2017
 *      Author: Andreas Nofen
 */


#include <ompl/base/State.h>
#include <opencv2/opencv.hpp>

#include <grid_map_ros/grid_map_ros.hpp>

#include <ros/package.h>

// TF transfrom
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include "traversability_estimation/TraversabilityMap.hpp"


class TraverabilityObjective : public ob::StateCostIntegralObjective
{
public:
	traversability_estimation::TraversabilityMap& traversabilityMap_;

    TraverabilityObjective(const ob::SpaceInformationPtr& si, traversability_estimation::TraversabilityMap traversabilityMap) :
        ob::StateCostIntegralObjective(si, true),
        traversabilityMap_(traversabilityMap)
    {
		
    }

    ob::Cost stateCost(const ob::State* s) const
    {
		const ob::SE2StateSpace::StateType *state_2d = state->as<ob::SE2StateSpace::StateType> ();
		const int &x (int (mapSize[0]-(state_2d->getX ()+mapsizeX/2)/0.06)), &y (int (mapSize[1]-(state_2d->getY ()+mapsizeY/2)/0.06)), &yaw (int ((180 / M_PI * state_2d->getYaw ())));
		Eigen::Array2i xyPos(x,y);


		std::vector<double> traverability;
		traverability.push_back(allowedMap.at("traversability_0",xyPos));
		traverability.push_back(allowedMap.at("traversability_45",xyPos));
		traverability.push_back(allowedMap.at("traversability_90",xyPos));
		traverability.push_back(allowedMap.at("traversability_135",xyPos));

		if(traverability[0] != traverability[0])
		{
			traverability.clear();
			//	ROS_INFO("compute traverability (traverability = %lf)",traverability[0]);

			traverability = traversabilityMap_->traversabilityAtPosition(xyPos);
			allowedMap = traversabilityMap_->getTraversabilityMap();

		}



		int size = 4;
		float deltaYaw = 180/size;
		float boundry = deltaYaw/2;

		for(int i=0;i<size;i++)
		{
			if(yaw>=-boundry && yaw<=boundry)
			{
				return ob::Cost(traverability[i]);

			}
			boundry += deltaYaw;
		}
		if(yaw<=-(180-deltaYaw/2) && yaw>=(180-deltaYaw/2))
		{
			return ob::Cost(traverability[0]);
		}
    }
};
