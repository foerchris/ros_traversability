


#include "traversability_estimation/OMPLPlannerWrapper.h"


#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>


#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;






OMPLPlanner::OMPLPlanner ()
{
	mapsizeX = 5;
	mapsizeY = 5;
	tfListener = std::unique_ptr<tf::TransformListener> (new tf::TransformListener);


	tf_prefix = ros::this_node::getNamespace();
	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);
	ROS_INFO("tf_prefix: %s .", tf_prefix.c_str());

	BASE_FRAME = tf_prefix + "/base_link";
	ODOM_FRAME = tf_prefix + "/odom";
	MAP_FRAME = tf_prefix + "/map";
	trasverability_map_0 = 0;
	trasverability_map_45 = 0;
	trasverability_map_90 = 0;
	resolution = 0.06;
	traversabilityMap_= nullptr;
}

OMPLPlanner::~OMPLPlanner ()
{
}



/**
 * Checks if state is a valid state in the given state space
 * @param state
 * @return
 */

class ValidityChecker : public ob::StateValidityChecker
{
public:
	traversability_estimation::TraversabilityMap& traversabilityMap_;
	Eigen::Array2i mapSize;
	double resolution;
	double mapsizeX;
	double mapsizeY;
	grid_map::GridMap allowedMap;
	std::vector<std::string> orientationMapNames;

    ValidityChecker(const ob::SpaceInformationPtr& si, traversability_estimation::TraversabilityMap& traversabilityMap) :
	//ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si),
        traversabilityMap_(traversabilityMap),
    	allowedMap(traversabilityMap.getTraversabilityMap())

    {

    //	allowedMap = traversabilityMap_.getTraversabilityMap();
		mapSize = allowedMap.getSize();
		resolution = allowedMap.getResolution();

		mapsizeX=mapSize[0]*resolution;
		mapsizeY=mapSize[1]*resolution;

		orientationMapNames.push_back("traversability_0");
		orientationMapNames.push_back("traversability_45");
		orientationMapNames.push_back("traversability_90");
		orientationMapNames.push_back("traversability_135");
	}
    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {
    	//bool bla = true;
    	/*if(this->traversability(state) > 0.5)
    	{

    	}*/
        return this->traversability(state) > 0.3;
        //return true;


    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double traversability(const ob::State* state) const
    {
		//std::cout<<"traversability"<<std::endl;

        const ompl::base::SE2StateSpace::StateType *state_2d = state->as<ompl::base::SE2StateSpace::StateType> ();
		const int x = static_cast<int> (mapSize[0] - (state_2d->getX () + mapsizeX / 2) / 0.06);
		const int y = static_cast<int> (mapSize[1] - (state_2d->getY () + mapsizeY / 2) / 0.06);
		const int yaw = static_cast<int> ((180 / M_PI * state_2d->getYaw ()));

		Eigen::Array2i xyPos(x,y);


		int size = 4;
		float deltaYaw = 180/size;
		float boundry = deltaYaw/2;
		//std::cout<<"allowedMap.at(traversability_0,xyPos)"<<allowedMap.at("traversability_0",xyPos)<<std::endl;

		std::string orientation;
		for(int i=0;i<size-1;i++)
		{
			if(boundry + i*deltaYaw <= fabs(yaw) && boundry + deltaYaw * (i+1) > fabs(yaw) )
			{
				double travValue = allowedMap.at(orientationMapNames[i+1],xyPos);
				if(travValue != travValue)
				{
					travValue = traversabilityMap_.traversabilityAtPosition(xyPos)[i];
					if(travValue != travValue)
					{
						std::cout<<"not exploraded area"<<std::endl;
						return 0.0;
					}
				}
			return travValue;
			}
		}

		if(0 <= fabs(yaw) && boundry > fabs(yaw) )
		{
			double travValue = allowedMap.at(orientationMapNames[0],xyPos);
			if(travValue != travValue)
			{
				travValue = traversabilityMap_.traversabilityAtPosition(xyPos)[0];
				if(travValue != travValue)
				{
					std::cout<<"not exploraded area"<<std::endl;
					return 0.0;
				}
			}
			return travValue;
		}
		else if(180 - boundry < fabs(yaw))
		{
			double travValue = allowedMap.at(orientationMapNames[0],xyPos);
			if(travValue != travValue)
			{
				travValue = traversabilityMap_.traversabilityAtPosition(xyPos)[0];
				if(travValue != travValue)
				{
					std::cout<<"not exploraded area"<<std::endl;
					return 0.0;
				}
			}
			return travValue;
		}
		return 0.0;
    }

};

class TraverabilityObjective : public ompl::base::StateCostIntegralObjective
{
public:
	ValidityChecker& validityChecker_;
    TraverabilityObjective(const ompl::base::SpaceInformationPtr& si, ValidityChecker& validityChecker) :
        ompl::base::StateCostIntegralObjective(si, true),
		validityChecker_(validityChecker)
    {

    }

    ompl::base::Cost stateCost(const ompl::base::State* state) const
    {
		return ob::Cost(validityChecker_.traversability(state));
    }
};

/**
 * Loads the calculated map of allowed angles
 * @param angle_map
 */
void OMPLPlanner::setTraversabilityMap (traversability_estimation::TraversabilityMap* traversabilityMap)
{
	traversabilityMap_ = traversabilityMap;

}

void OMPLPlanner::setTraversabilityGridMap(grid_map::GridMap travGridMap)
{
	allowedMap = travGridMap;
}



/**
 * Plans path from the zero position to the given goal using RRT* in a simple dubins state space.
 * @param goal_d - x,y position of the goal
 * @param waypoints - x,y position and orientation of points on the calculated path
 */
void OMPLPlanner::plan (pose goal_d, std::vector<pose> &waypoints)
{

	ROS_INFO("plan");
	float planRadius = 0.2;

	//ob::StateSpacePtr space(new ob::ReedsSheppStateSpace);
	//ob::StateSpacePtr space (new ob::DubinsStateSpace);
	ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>());

	space = std::make_shared<ob::DubinsStateSpace>(planRadius);

	ob::RealVectorBounds bounds (2);

	allowedMap = traversabilityMap_->getTraversabilityMap();

	mapSize = allowedMap.getSize();
	resolution = allowedMap.getResolution();

	ros::Rate rate(20);
	mapsizeX=mapSize[0]*resolution;
	mapsizeY=mapSize[1]*resolution;

	printf("mapsizeX: %lf", mapsizeX);
	printf("mapsizeY: %lf", mapsizeY);
	//allowedMap.get
	bounds.setLow (0, -mapsizeX/2); //1.14
	bounds.setLow (1, -mapsizeY/2);
	bounds.setLow (2, 0.0 * boost::math::constants::pi<double> ());

	bounds.setHigh (0, mapsizeX/2); //1.14
	bounds.setHigh (1, mapsizeY/2);
	bounds.setHigh (2, boost::math::constants::pi<double> ()/2);

	space->as<ob::SE2StateSpace> ()->setBounds (bounds);

	// define a simple setup class
	//og::SimpleSetup ss (space);

   // Construct a space information instance for this state space
     auto si(std::make_shared<ob::SpaceInformation>(space));


	// set state validity checking for this space
	//ss.setStateValidityChecker (boost::bind (&OMPLPlanner::isStateValid, this, _1));
    auto validityChecker = std::make_shared<ValidityChecker>(si, *traversabilityMap_);
	si->setStateValidityChecker(validityChecker);
	//si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));

	//ss.setStateValidityChecker (boost::bind (&OMPLPlanner::isStateValid, this, _1));

	si->setup();


	// set the start and goal states
	ob::ScopedState<> start (space), goal (space);

	pose startPose;
	startPose.x = 0.0;
	startPose.y = 0.0;
	startPose.yaw = 0.0;
	robotToGlobaleTransform(startPose);
	std::cout << "\n startPose:  x: " <<startPose.x<<", y: " <<startPose.y <<", yaw: " <<startPose.yaw<< std::endl;

	start[0] = startPose.x;
	start[1] = startPose.y;
	start[2] = startPose.yaw;

	robotToGlobaleTransform(goal_d);
	std::cout << "\n goal_d:  x: " <<goal_d.x<<", y: " <<goal_d.y <<", yaw: " <<goal_d.yaw<< std::endl;


	goal[0] = goal_d.x;
	goal[1] = goal_d.y;
	goal[2] = goal_d.yaw;

	//ss.setStartAndGoalStates (start, goal);

	// Create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);


    //ss.setOptimizationObjective(boost::bind (&OMPLPlanner::getTraversabilityObjectivconst, this, _1));

	// Create the optimization objective specified by our command-line argument.
    // This helper function is simply a switch statement.

    auto lengthObjective = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    auto travObjective = std::make_shared<TraverabilityObjective>(si, *validityChecker);

    auto multiObjective = std::make_shared<ob::MultiOptimizationObjective>(si);

    multiObjective->addObjective (lengthObjective, 5.0);
    multiObjective->addObjective (travObjective, 5.0);


    pdef->setOptimizationObjective(multiObjective);

	//ob::PlannerPtr planner (new og::RRTstar (si.getSpaceInformation ()));

	//ss.setPlanner (planner);

	// Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr optimizingPlanner = std::make_shared<og::RRTstar>(si);


	optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

	// this call is optional, but we put it in to get more output information
	//ss.getSpaceInformation ()->setStateValidityCheckingResolution (0.01);
	//ss.setup ();

	// attempt to solve the problem within 30 seconds of planning time
	//ob::PlannerStatus solved = ss.solve (2);
     ob::PlannerStatus solved = optimizingPlanner->solve(20);

	if (solved)
	{
		std::cout << "Found solution:" << std::endl;
		 std::cout
		             << optimizingPlanner->getName()
		             << " found a solution of length "
		             << pdef->getSolutionPath()->length()
		             << " with an optimization objective value of "
		             << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

		 std::string outputFile = ros::package::getPath("traversability_estimation")+"/bla.txt";

		// ros::package::getPath("traversability_estimation");
		 std::ofstream outFile(outputFile.c_str());
         std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->
             printAsMatrix(outFile);
         outFile.close();
         auto geoPath = *std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());

         double length = geoPath.length ();
     		std::cout << "length: " <<length<< std::endl;

     		geoPath.interpolate (length / 0.05 + 1);

     		ob::ScopedState<> s (space);
     		for (std::size_t i = 0; i < geoPath.getStateCount (); i++)
     		{
     			s = geoPath.getState (i);
     			pose wayPose;
     			wayPose.x = s.reals ().at(0);
     			wayPose.y = s.reals ().at(1);
     			wayPose.yaw = s.reals ().at(2);
     			waypoints.push_back (wayPose);
     		}
	}
	else
	std::cout << "No solution found" << std::endl;

}

void OMPLPlanner::globaleToRobotTransform(pose &Pose)
{
	// TF transformation of the Point which is nearest to the robot
	const ros::Time& scanTimeStamp = ros::Time (0);
	try
	{
		tfListener->waitForTransform (BASE_FRAME, MAP_FRAME, scanTimeStamp, ros::Duration (3.0));
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}

	tf::Vector3 origin;

	origin.setX(Pose.x);
	origin.setY(Pose.y);
	origin.setZ(0);

	tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, Pose.yaw);

	tf::StampedTransform transform;

	tf::Stamped<tf::Pose> transPose (tf::Pose( quat,origin), scanTimeStamp, MAP_FRAME);

	tf::Stamped<tf::Pose> poseTransformed;

	try
	{
		tfListener->transformPose(BASE_FRAME, scanTimeStamp, transPose, MAP_FRAME, poseTransformed);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR (" Point xyz %s", ex.what ());
		return;
	}
	origin=poseTransformed.getOrigin();
	quat=poseTransformed.getRotation();
	Pose.x=origin.getX();
	Pose.y=origin.getY();
	Pose.yaw= tf::getYaw(quat);
}

void OMPLPlanner::robotToGlobaleTransform(pose &Pose)
{
	// TF transformation of the Point which is nearest to the robot
	const ros::Time& scanTimeStamp = ros::Time (0);
	try
	{
		tfListener->waitForTransform (MAP_FRAME,BASE_FRAME, scanTimeStamp, ros::Duration (3.0));
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}

	tf::Vector3 origin;

	origin.setX(Pose.x);
	origin.setY(Pose.y);
	origin.setZ(0);

	tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, Pose.yaw);

	tf::StampedTransform transform;

	tf::Stamped<tf::Pose> transPose (tf::Pose( quat,origin), scanTimeStamp, BASE_FRAME);

	tf::Stamped<tf::Pose> poseTransformed;

	try
	{
		tfListener->transformPose(MAP_FRAME, scanTimeStamp, transPose, BASE_FRAME, poseTransformed);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR (" Point xyz %s", ex.what ());
		return;
	}
	origin=poseTransformed.getOrigin();
	quat=poseTransformed.getRotation();
	Pose.x=origin.getX();
	Pose.y=origin.getY();
	Pose.yaw= tf::getYaw(quat);
}
