


#include "traversability_estimation/OMPLPlannerWrapper.h"

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
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
bool OMPLPlanner::isStateValid (const ob::State *state)
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
			if(traverability[i]>0.5)
			{
				return true;
			}
		}
		boundry += deltaYaw;
	}
	if(yaw<=-(180-deltaYaw/2) && yaw>=(180-deltaYaw/2))
	{
		if(traverability[0]>0.5)
		{
			return true;
		}
	}
	return false;
}

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
	float planRadius = 0.5;

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
	og::SimpleSetup ss (space);


	// set state validity checking for this space
	ss.setStateValidityChecker (boost::bind (&OMPLPlanner::isStateValid, this, _1));

	ob::PlannerPtr planner (new og::RRTstar (ss.getSpaceInformation ()));
	ss.setPlanner (planner);


	// set the start and goal states
	ob::ScopedState<> start (space), goal (space);

	pose startPose;
	startPose.x = 0.0;
	startPose.y = 0.0;
	startPose.yaw = 0.0;
	robotToGlobaleTransform(startPose);
	std::cout << "startPose:  x: " <<startPose.x<<", y: " <<startPose.y <<", z: " <<startPose.z<< std::endl;

	start[0] = startPose.x;
	start[1] = startPose.y;
	start[2] = startPose.yaw;

	robotToGlobaleTransform(goal_d);
	std::cout << "goal_d:  x: " <<goal_d.x<<", y: " <<goal_d.y <<", z: " <<goal_d.z<< std::endl;


	goal[0] = goal_d.x;
	goal[1] = goal_d.y;
	goal[2] = goal_d.yaw;

	ss.setStartAndGoalStates (start, goal);


	// this call is optional, but we put it in to get more output information
	ss.getSpaceInformation ()->setStateValidityCheckingResolution (0.01);
	ss.setup ();

	// attempt to solve the problem within 30 seconds of planning time
	ob::PlannerStatus solved = ss.solve (2);


	if (solved)
	{
		std::cout << "Found solution:" << std::endl;

		ss.simplifySolution ();
		og::PathGeometric path = ss.getSolutionPath ();
		double length = path.length ();
		std::cout << "length: " <<length<< std::endl;

		path.interpolate (length / 0.05 + 1);

		ob::ScopedState<> s (space);
		for (std::size_t i = 0; i < path.getStateCount (); i++)
		{
			s = path.getState (i);
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
