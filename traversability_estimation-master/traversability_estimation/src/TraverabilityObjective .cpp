


#include "traversability_estimation/TraverabilityObjective.h"

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;


TraverabilityObjective(const ob::SpaceInformationPtr& si, traversability_estimation::TraversabilityMap, traversabilityMap_) :
			ob::StateCostIntegralObjective(si, true),
			traversability_estimation::TraversabilityMap* traversabilityMap_;
{
}

    
TraverabilityObjective::~TraverabilityObjective ()
{
}

ob::Cost stateCost(const ob::State* s) const
{
	return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
}
