#include "ContourPathPlanner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;



ContourPathPlanner::ContourPathPlanner()
{
    markerPublisher = thisNodeHandel.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 20);
	/*while (markerPublisher.getNumSubscribers() < 1)
	{
		ROS_WARN_ONCE("Please create a subscriber to the marker");
		sleep(1);
	}*/
    pathForwardView = true;
    pathReversOriantation = true;
}


ContourPathPlanner::~ContourPathPlanner()
{

}


visualization_msgs::Marker ContourPathPlanner::createMarker (std::string ns, int id, double x, double y,  double r = 1.0, double g = 0.0, double b = 0.0, double a = 1.0)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = "map";
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

void ContourPathPlanner::plan(const ompl::base::StateSpacePtr& space, bool easy)
{
    ob::ScopedState<> start(space), goal(space);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    if (easy)
        bounds.setHigh(18);
    else
    {
        bounds.high[0] = 6;
        bounds.high[1] = .6;
    }
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    if (easy)
    {
        start[0] = start[1] = 1.; start[2] = 0.;
        goal[0] = goal[1] = 17; goal[2] = -.99*M_PI;
    }
    else
    {
        start[0] = start[1] = .5; start[2] = .5*M_PI;;
        goal[0] = 5.5; goal[1] = .5; goal[2] = .5*M_PI;
    }
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss.setup();
    ss.print();

    // attempt to solve the problem within 30 seconds of planning time
    ob::PlannerStatus solved = ss.solve(30.0);

    if (solved)
    {
        std::vector<double> reals;

        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        og::PathGeometric path = ss.getSolutionPath();
        path.interpolate(1000);
        path.printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

void ContourPathPlanner::printTrajectory(const ompl::base::StateSpacePtr& space,  const std::vector<double>& pt)
{

    ob::ScopedState<> from(space);
	ob::ScopedState<> to(space);
	ob::ScopedState<> s(space);

    std::vector<double> reals;

    from[0] = pt[0];
    from[1] = pt[1];
    from[2] = pt[2];

    to[0] = pt[3];
    to[1] = pt[4];
    to[2] = pt[5];

    const unsigned int num_pts = (space->distance(from(), to()) / 0.1);

    std::cout << "Number of points " <<  num_pts << std::endl;

    std::cout << "distance: " << space->distance(from(), to()) << "\npath:\n";

    visualization_msgs::MarkerArray markerArray;

    for (unsigned int i=0; i<=num_pts; ++i)
    {
        space->interpolate(from(), to(), (double)i/num_pts, s());
        reals = s.reals();
        std::cout << "path " << reals[0] << ' ' << reals[1] << ' ' << reals[2] << ' ' << std::endl;
        pose creatPose;

        creatPose.x=reals[0];
        creatPose.y=reals[1];
        creatPose.yaw=reals[2];

        Poses.push_back(creatPose);

        int id_seperator = pt.at (6);

        markerArray.markers.push_back (createMarker("static line", i + id_seperator, reals[0], reals[1], 1.0, 1.0, 0.0));
    }

    markerPublisher.publish(markerArray);
}

void ContourPathPlanner::printPolyTrajectory(const ompl::base::StateSpacePtr& space,  const std::vector<pose>& pt)
{
    ob::ScopedState<> from(space), to(space), s(space);
    std::vector<double> reals;
	visualization_msgs::MarkerArray markerArray;

    for(std::size_t i=0; i<pt.size()-1; i++)
    {
    	from[0] = pt.at(i).x;
    	from[1] = pt.at(i).y;
    	from[2] = pt.at(i).yaw;
	    std::cout << "from"<<i+1<<": " << from[0] << ' ' << from[1] << ' ' << from[2] << ' ' << std::endl;

    	to[0] = pt.at(i+1).x;
    	to[1] = pt.at(i+1).y;
    	to[2] = pt.at(i+1).yaw;
	    std::cout << "to"<<i+1<<": " << to[0] << ' ' << to[1] << ' ' << to[2] << ' ' << std::endl;

    	const unsigned int num_pts = (space->distance(from(), to()) / 0.02);

    	//std::cout << "Number of points " <<  num_pts << std::endl;

    	//std::cout << "distance: " << space->distance(from(), to()) << "\npath:\n";

    	for (unsigned int j=0; j<=num_pts; ++j)
    	{
    	    space->interpolate(from(), to(), (double)j/num_pts, s());
    	    reals = s.reals();
    	    std::cout << "path"<<i+1<<": " << reals[0] << ' ' << reals[1] << ' ' << reals[2] << ' ' << std::endl;

    	    pose creatPose;

    	    creatPose.x=reals[0];
    	    creatPose.y=reals[1];
    	    creatPose.yaw=reals[2];

    	    Poses.push_back(creatPose);

    	    int id_seperator = 1;

    	    markerArray.markers.push_back (createMarker("static line", i*(num_pts)+(j + id_seperator), reals[0], reals[1], 1.0, 1.0, 0.0));
    	}
    }
    markerPublisher.publish(markerArray);

}
// New Print function for Circles 16.02.18
void ContourPathPlanner::printCircleTrajectory(const ompl::base::StateSpacePtr& space,  const std::vector<double>& pt)
{
    ob::ScopedState<> from(space), to(space), s(space);
    std::vector<double> reals;

    from[0] = pt[0];
    from[1] = pt[1];
    from[2] = pt[2];

    to[0] = pt[3];
    to[1] = pt[4];
    to[2] = pt[5];

    // radius = pt[7]	orientation = pt[8]
    // angles from[2] and to [2] can be anything till here, have to be calculated and overwritten in the following

    double radius = pt[7];
    int orientation = pt[8];


    double distance_points_circle = std::hypot(to[0] - from[0], to [1]- from[1]);
    double angle_alpha = asin((distance_points_circle/2) / radius);
    double orientation_axis = atan2(to [1]- from[1], to[0] - from[0]);

    if (orientation == 1)
    {
    	from[2] = orientation_axis + angle_alpha;
    	to[2] = orientation_axis - angle_alpha;
    	std::cout << "left " << std::endl;
    }

    else if (orientation == -1)
    {
    	from[2] = orientation_axis - angle_alpha;
    	to[2] = orientation_axis + angle_alpha;
    	std::cout << "right " << std::endl;
    }

    const unsigned int num_pts = (space->distance(from(), to()) / 0.02);
    std::cout << "Number of points " <<  num_pts << std::endl;

    std::cout << "distance: " << space->distance(from(), to()) << "\npath:\n";

    visualization_msgs::MarkerArray markerArray;

    for (unsigned int i=0; i<=num_pts; ++i)
    {
        space->interpolate(from(), to(), (double)i/num_pts, s());
        reals = s.reals();
   //     std::cout << "path " << reals[0] << ' ' << reals[1] << ' ' << reals[2] << ' ' << std::endl;

        int id_seperator = pt.at (6);
        //trackingControl->referencePath(Poses,true);
        markerArray.markers.push_back (createMarker("static circle", i + id_seperator, reals[0], reals[1], 1.0, 0.0, 0.0));

    }
    markerPublisher.publish(markerArray);
}



bool ContourPathPlanner::followPath(chosenFollowAlg followAlg, bool resetPath)
{

		switch (followAlg)
		{
			case SimpleYDeviationControl:
				if(startPoses.size()>0)
				{
					trackingControl->simpleYDeviationControl(startPoses.at(0).x,startPoses.at(0).y,startPoses.at(0).z,true);
				}
				return true;
				break;
			case SimpleThetaDeviationControl:
				if(startPoses.size()>1)
				{
					trackingControl->simpleThetaDeviationControl(startPoses.at(0).x,startPoses.at(0).y,startPoses.at(0).z,true);
				}
				return true;
				break;
			case ReferencePathControl:
				if(Poses.size()!=0)
				{
					return trackingControl->referencePath(Poses,resetPath);
				}
				break;
			default:
				return false;
				break;
		}
		return false;
}
void ContourPathPlanner::startRobotMovementfromCont(bool startMov)
{
	trackingControl->startRobotMovement(startMov);
}

void ContourPathPlanner::forwardBackwarMode(const bool& forwardView, const bool& reversOriantation,int direction)
{

	if(forwardView && reversOriantation)
	{
		pathForwardView = true;
		pathReversOriantation = true;
		// driving forward with front view
		trackingControl->alginCamera(forwardView);
		trackingControl->setOriantation(1);
	}
	else if(!forwardView && reversOriantation)
	{
		pathForwardView = false;
		pathReversOriantation = true;
		// driving forward with rear view
		trackingControl->alginCamera(forwardView);
		trackingControl->setOriantation(1);
	}
	else if(forwardView && !reversOriantation)
	{
		pathForwardView = true;
		pathReversOriantation = false;
		// driving backward with front view
		trackingControl->alginCamera(forwardView);
		trackingControl->setOriantation(-1);
	}
	else
	{
		pathForwardView = false;
		pathReversOriantation = false;
		// driving backward with rear view
		trackingControl->alginCamera(forwardView);
		trackingControl->setOriantation(-1);
	}

	trackingControl->setDirection(direction);
}

void ContourPathPlanner::setPathPoints(std::vector<pose> inputPoints)
{
	startPoses.clear();
    startPoses=inputPoints;
}


void ContourPathPlanner::clcPathToSinglePoint(double planRadius)
{
    std::vector<pose> pt;
    Poses.clear();
    pose zeroPose;
    zeroPose.x =0;
    zeroPose.y =0;
    zeroPose.yaw =0;
    int view=1;
    int orientation=1;
    if(pathForwardView && pathReversOriantation)
    {
    	view=1;
    	orientation=1;
    }
    else if(!pathForwardView && pathReversOriantation)
    {
    	view=1;
        orientation=-1;
    }
    else if(pathForwardView && !pathReversOriantation)
    {
    	view=-1;
        orientation=1;
        zeroPose.yaw =M_PI;
    }
    else
    {
    	view=-1;
        orientation=-1;
        zeroPose.yaw =M_PI;
    }

    pt.push_back(zeroPose);
    pt.push_back(startPoses.at(0));
    //pt.at(1).x=pt.at(1).x;
    for(std::size_t i=0; i<pt.size();i++)
    {
    	trackingControl->robotToGlobaleTransform(pt.at(i));
    }
	try
	{
		visualization_msgs::MarkerArray markerArray;
	    ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>());
	    space = std::make_shared<ob::DubinsStateSpace>(planRadius);
	    plan (space, true);
		printPolyTrajectory(space, pt);
	}
	catch(std::exception& e)
	{
	    std::cerr << "error: " << e.what() << "\n";
	}
	catch(...) {
	    std::cerr << "Exception of unknown type!\n";
	}
}

void ContourPathPlanner::clcPathToPoints(double planRadius)
{
    std::vector<pose> pt;
    pose zeroPose;
    zeroPose.x =0;
    zeroPose.y =0;
    zeroPose.yaw =0;
    pt.push_back(zeroPose);
    int view=1;
    int orientation=1;
    if(pathForwardView && pathReversOriantation)
    {
    	view=1;
    	orientation=1;
    }
    else if(!pathForwardView && pathReversOriantation)
    {
    	view=1;
        orientation=-1;
    }
    else if(pathForwardView && !pathReversOriantation)
    {
    	view=-1;
        orientation=1;
        zeroPose.yaw =M_PI;

    }
    else
    {
    	view=-1;
        orientation=-1;
        zeroPose.yaw =M_PI;
    }

    for(size_t i=0;i<startPoses.size();i++)
    {
    	//pose invPos=startPoses.at(i);
    	//invPos.x=orientation*invPos.x;
    	//invPos.y=view*invPos.y;
        pt.push_back(startPoses.at(i));
    }

	for(std::size_t i=0; i<pt.size();i++)
	{
		trackingControl->robotToGlobaleTransform(pt.at(i));
	}

	try
	{
		visualization_msgs::MarkerArray markerArray;
	    ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>());
	    space = std::make_shared<ob::DubinsStateSpace>(planRadius);
	    plan (space, true);
		printPolyTrajectory(space, pt);
	}
	catch(std::exception& e)
	{
	    std::cerr << "error: " << e.what() << "\n";
	}
	catch(...) {
	    std::cerr << "Exception of unknown type!\n";
	}
}

