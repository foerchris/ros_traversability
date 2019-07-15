/*
 * LineFollowAlgorithm.cpp
 *
 *  Created on: Oct 5, 2017
 *      Author: chfo
 */


#include "TrackingControl.h"

#include <stdio.h>
#include <cmath>


using namespace std;
//#define TrackingControl_SIMULATOR_ENABLED true

TrackingControl::TrackingControl(ros::NodeHandle nh, double turn_speed, double fast_speed, double init_speed)
{

	BASE_FRAME = "/base_link";
	MAP_FRAME = "/map";
	ODOM_FRAME = "/odom";
	tf_prefix = "//GETjag1";
	
	tf_prefix = ros::this_node::getNamespace();
	if(tf_prefix == "/")
	{
		tf_prefix = "//GETjag1";
	}
	
	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	istringstream iss (tf_prefix.substr(6, tf_prefix.size()));
	int robot_number = 1;
	iss >> robot_number;

	BASE_FRAME = tf_prefix + BASE_FRAME;
	MAP_FRAME = tf_prefix + MAP_FRAME;
	ODOM_FRAME = tf_prefix + ODOM_FRAME;

    markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 20);
    camAnglePitchPub =nh.advertise<std_msgs::Float64>("sensorhead_pitch_controller/command",1);
    camAngleYawPub =nh.advertise<std_msgs::Float64>("sensorhead_yaw_controller/command",1);
    odomSub = nh.subscribe<nav_msgs::Odometry> ("odom", 1, &TrackingControl::odomCallback,this);
	forwardMode=1;
    robotStartStopSub = nh.subscribe<std_msgs::Bool> ("start_stop_robot", 1, &TrackingControl::startStopCallback,this);

	// ROS Publisher
	velPub = nh.advertise<geometry_msgs::Twist>( "cmd_vel",1);	// ROS services
	srvFollow = nh.advertiseService("follow_object_in_image", &TrackingControl::followObjectCallback, this);

	save=0;
	count=0;
	robotStartStop=true;
	robotSpeed = 0.2;
	currentTrackingState=START;
	Kp = 0.5;
	Ki = 5;
	Kd = 0.1;

	direction=1;
	//ROS timer
    currentTime = ros::Time::now().toSec();
	previousTime = currentTime;
	timer = nh.createTimer(ros::Duration(0.2),&TrackingControl::timerCallback,this);

	prozentAngVel=0;
	prozentLinVel=0;

	// distiguish between forward driving (1) and backward driving (-1)

	currentAngle=0;
	currentX=0;
	currentY=0;
	lineFollowStat= start;
	dsx=0;
	dsy=0;
	dAngle=0;


	tfListener = std::unique_ptr<tf::TransformListener> (new tf::TransformListener);

    // dynamic reconfigure
	static dynamic_reconfigure::Server<robot_navigation::TrackingControlConfig> serverTrackCon (ros::NodeHandle ("~/TrackingControl"));
	static dynamic_reconfigure::Server<robot_navigation::TrackingControlConfig>::CallbackType f2;
	f2 = boost::bind(&TrackingControl::reconfigureCallback,this,_1,_2);
	serverTrackCon.setCallback(f2);
	runTime = ros::Time::now().toSec();
}

TrackingControl::~TrackingControl()
{

}

void TrackingControl::alginCamera(const bool& choseFrontRearView)
{
	std::cout<<"alginCamera"<<std::endl;
	if(choseFrontRearView==true)
	{
		camAnglePitch.data=0.34;
		camAngleYaw.data=0;
	}
	else
	{
		camAnglePitch.data=0.5;
		camAngleYaw.data=3.141;
	}
	camAnglePitchPub.publish (camAnglePitch);
	camAngleYawPub.publish (camAngleYaw);
}
void TrackingControl::setOriantation(const int& choseForBackMode)
{
	forwardMode=choseForBackMode;
}

void TrackingControl::setDirection(const int& direc)
{
	direction=direc;
}

void TrackingControl::simpleYDeviationControl(double xGoal, double yGoal , double zGoal,bool lineFound)
{
	yGoal=forwardMode*yGoal;

	geometry_msgs::Twist velocities;
	if(robotStartStop)
	{
		if (!lineFound) //Line lost
		{
		/*	velocities.linear.x = 0.0;

			if(velocities.angular.z >= 0.0)
			{
				velocities.angular.z =  MAX_ANGULAR_VEL;
			}else
			{
				velocities.angular.z = -MAX_ANGULAR_VEL;
			}*/
		}
		else
		{
			currentTime = ros::Time::now().toSec();
			double dt = (currentTime - previousTime);
			double currentDelta=yGoal;
			double errorTerm = currentDelta;
			double pidAngle = Kp*errorTerm + Ki * dt * errorTerm + Kd*errorTerm/ dt;
			previousTime = currentTime;
			// Set Angular Velocitie to follow line
			if(pidAngle<=1 && pidAngle>=-1)
			{
				velocities.angular.z = pidAngle;
				velocities.linear.x = direction*robotSpeed;
				runTime = ros::Time::now().toSec();
			}
		}
		velPub.publish (velocities);
	}
}

void TrackingControl::simpleThetaDeviationControl(double xGoal, double yGoal , double zGoal,bool lineFound)
{
	geometry_msgs::Twist velocities;
	if(robotStartStop)
	{
		if (!lineFound) //Line lost
		{
			velocities.linear.x = 0.0;

			if(velocities.angular.z >= 0.0)
			{
				velocities.angular.z =  MAX_ANGULAR_VEL;
			}else
			{
				velocities.angular.z = -MAX_ANGULAR_VEL;
			}
		}
		else
		{
			currentTime = ros::Time::now().toSec();
			double thetaRef=atan2(yGoal,xGoal);
			currentTime = ros::Time::now().toSec();
			double dt = (currentTime - previousTime);
			double currentDelta=thetaRef;
			double errorTerm = currentDelta/M_PI*2;
			double pidAngle = Kp*errorTerm + Ki * dt * errorTerm + Kd*errorTerm/ dt;
			previousTime = currentTime;
			if(pidAngle<=1 && pidAngle>=-1)
			{
				double linearVel = (prozentLinVel*MAX_LINEAR_VEL * (1 -fabs(pidAngle)) ); //45 because this will be the maximum of reression
				double angularVel =  1 * (pidAngle) * prozentAngVel*MAX_ANGULAR_VEL; //45 because this will be the maximum of reression            // Set Angular Velocitie to follow line
			    velocities.angular.z = angularVel;
			    velocities.linear.x = forwardMode*linearVel;
			    runTime = ros::Time::now().toSec();
			}
		}
	}
	else
	{
		velocities.angular.z = 0;
		velocities.linear.x = 0;
	}
	velPub.publish (velocities);
}



bool TrackingControl::pointToPointCalc(const std::vector<pose>& globalCoordinates, bool reset)
{
    visualization_msgs::MarkerArray markerArray;
    bool terminate=true;
	double ka=0.1;
	double Kx=2;
	double Ky=ka*ka/100;
	double Ktheta=16*Ky*Ky;

	static std::size_t selectedPoint;

	if(reset)
	{
		selectedPoint=0;
	}
	pose errorPose;
	std::vector<pose> robotCoordinates = globalCoordinates;

	for(std::size_t i=0;i<robotCoordinates.size();i++)
	{
			globaleToRobotTransform(robotCoordinates.at(i));
	}
	std::vector<pose> updatedPoints;
	for(std::size_t i=selectedPoint;i<robotCoordinates.size();i++)
	{
		if(robotCoordinates.at(i).x>0)
		{
			terminate=false;
			selectedPoint=i;
			if(selectedPoint>0 && selectedPoint<robotCoordinates.size()-1)
			{
				updatedPoints.push_back(globalCoordinates.at(i-1));
				updatedPoints.push_back(globalCoordinates.at(i));
				updatedPoints.push_back(globalCoordinates.at(i+1));
			}
			else if (selectedPoint<=0)
			{
				updatedPoints.push_back(globalCoordinates.at(i));
				updatedPoints.push_back(globalCoordinates.at(i+1));
				updatedPoints.push_back(globalCoordinates.at(i+2));
			}
			else
			{
				updatedPoints.push_back(globalCoordinates.at(i-2));
				updatedPoints.push_back(globalCoordinates.at(i-1));
				updatedPoints.push_back(globalCoordinates.at(i));
			}
			break;
		}
	}
	if(terminate)
	{
		velocities.linear.x = 0;
		velocities.angular.z = 0;
		velPub.publish (velocities);
		return false;
	}
	double tangentialVelocity, angularVelocity,curvature;

	curvature=clcCurvature(updatedPoints);
    markerArray.markers.push_back (createMarker("static circle", 1, globalCoordinates.at(selectedPoint).x, globalCoordinates.at(selectedPoint).y, 1.0, 0.0,1.0, 1.0));
    markerPublisher.publish(markerArray);


	errorPose.x=robotCoordinates.at(selectedPoint).x;
	errorPose.y=robotCoordinates.at(selectedPoint).y;
	errorPose.yaw=robotCoordinates.at(selectedPoint).yaw;

	tangentialVelocity= robotSpeed;

	if(curvature>=2)
	{
		tangentialVelocity=0.1;
	}
/*
	double v = tangentialVelocity*cos(errorPose.yaw)+Kx*errorPose.x;

	double omega = angularVelocity + tangentialVelocity*(Ky*errorPose.y+Ktheta*sin(errorPose.yaw));
	*/
	double v = tangentialVelocity*cos(errorPose.yaw)+Kp*errorPose.x;
	angularVelocity=curvature*tangentialVelocity;

	double omega = angularVelocity + tangentialVelocity*(Ki*errorPose.y+Kd*sin(errorPose.yaw));

	velocities.linear.x = v;
	velocities.angular.z = omega;

	if(robotStartStop)
	{
		velPub.publish (velocities);
	}

	return true;

}

bool TrackingControl::referencePath(const std::vector<pose> &globalCoordinates, bool reset)
{
    visualization_msgs::MarkerArray markerArray;
    bool terminate=true;
	static std::size_t selectedPoint;

	if(reset)
	{
		selectedPoint=0;
	}
	pose errorPose;
	std::vector<pose> robotCoordinates = globalCoordinates;

	for(std::size_t i=0;i<robotCoordinates.size();i++)
	{
		globaleToRobotTransform(robotCoordinates.at(i));
		robotCoordinates.at(i).x=forwardMode*robotCoordinates.at(i).x;
		robotCoordinates.at(i).y=forwardMode*robotCoordinates.at(i).y;
	}
	std::vector<pose> updatedPoints;
	for(std::size_t i=selectedPoint;i<robotCoordinates.size();i++)
	{
		if(robotCoordinates.at(i).x>0)
		{
			terminate=false;
			selectedPoint=i;
			if(selectedPoint>0 && selectedPoint<robotCoordinates.size()-1)
			{
				updatedPoints.push_back(globalCoordinates.at(i-1));
				updatedPoints.push_back(globalCoordinates.at(i));
				updatedPoints.push_back(globalCoordinates.at(i+1));
			}
			else if (selectedPoint<=0)
			{
				updatedPoints.push_back(globalCoordinates.at(i));
				updatedPoints.push_back(globalCoordinates.at(i+1));
				updatedPoints.push_back(globalCoordinates.at(i+2));
			}
			else
			{
				updatedPoints.push_back(globalCoordinates.at(i-2));
				updatedPoints.push_back(globalCoordinates.at(i-1));
				updatedPoints.push_back(globalCoordinates.at(i));
			}
			for(std::size_t j=0; j< updatedPoints.size(); j++)
			{
				if(std::isnan(updatedPoints.at(j).yaw))
				{
					updatedPoints.at(j).yaw=0;
				}
			}
			break;
		}
	}

	if(terminate)
	{
		velocities.linear.x = 0;
		velocities.angular.z = 0;
		velPub.publish (velocities);
		return false;
	}

	double tangentialVelocity, angularVelocity,curvature;

	curvature=clcCurvature(updatedPoints);

	markerArray.markers.push_back (createMarker("static circle", 1, globalCoordinates.at(selectedPoint).x, globalCoordinates.at(selectedPoint).y, 1.0, 0.0,1.0, 1.0));
    markerArray.markers.push_back (createMarker("static circle", 1, globalCoordinates.at(selectedPoint).x, globalCoordinates.at(selectedPoint).y, 1.0, 0.0,1.0, 1.0));
    markerPublisher.publish(markerArray);


	errorPose.x=robotCoordinates.at(selectedPoint).x;
	errorPose.y=robotCoordinates.at(selectedPoint).y;
	errorPose.yaw=robotCoordinates.at(selectedPoint).yaw;

	if(std::isnan(errorPose.yaw))
	{
		errorPose.yaw=0;
	}


	tangentialVelocity= robotSpeed;

	if(curvature>=2)
	{
		tangentialVelocity=0.1;
	}

	angularVelocity=curvature*tangentialVelocity;
	double v1 = -K1*errorPose.x;
	double v2 = -sgn(tangentialVelocity)*K2*errorPose.y - K3*errorPose.yaw;

	double u1 = tangentialVelocity*cos(errorPose.yaw)-v1;
	double u2 = angularVelocity-v2;

	velocities.linear.x = direction*u1;
	velocities.angular.z = u2;

	if(robotStartStop)
	{
		if(	selectedPoint<=2 || selectedPoint >= robotCoordinates.size()-3)
		{
			velocities.angular.z = 0;
		}

        velPub.publish (velocities);
	}

	return true;

}
bool TrackingControl::pointDistanzCheak(const pose &globalCoordinates)
{

	pose robotCoordinates=globalCoordinates;


	globaleToRobotTransform(robotCoordinates);

	if(robotCoordinates.x>0)
	{
		return true;
	}
	else
	{
		return false;
	}

}
double TrackingControl::clcCurvature(std::vector<pose> curvPoints)
{
		pose p1 = curvPoints.at(0);
		pose p2 = curvPoints.at(1);
		pose p3 = curvPoints.at(2);

		return 2*area2(p1,p2,p3)/(distanceTo(p1,p2)*distanceTo(p2,p3)*distanceTo(p3,p1));

}


double TrackingControl::area2(pose p1, pose p2 , pose p3)
{
    return (p2.x-p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);
}
double TrackingControl::distanceTo(pose p1, pose p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}

int TrackingControl::sgn(double x)
{
  if (x > 0.0)
    return 1.0;
  else if (x < 0.0)
    return -1.0;
  else
    return 0.0;
}

void TrackingControl::globaleToRobotTransform(pose &Pose)
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

void TrackingControl::robotToGlobaleTransform(pose &Pose)
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

void TrackingControl::setLinearVel(double vel)
{
	geometry_msgs::Twist velocities;
	if(robotStartStop)
	{
		velocities.linear.x = vel;
	}
	else
	{
		velocities.linear.x = 0;
	}
	velocities.angular.z = 0;
	velPub.publish (velocities); //move the robot
}

double TrackingControl::calcTheta(pose P1, pose P2)
{
	double theta=atan2(P2.y-P1.y,P2.x-P1.x);
	return theta;
}

bool TrackingControl::compareCord(std::vector<double> P1, std::vector<double> P2)
{
	double x = P2[0]-P1[0];
	double y = P2[1]-P1[1];
	if(sqrt(x*x+y*y)>=0.15)
	{
		return true;
	}
	else
	{
		return false;
	}
}

visualization_msgs::Marker TrackingControl::createMarker (std::string ns, int id, double x, double y,  double r, double g , double b , double a )
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = MAP_FRAME;
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
void TrackingControl::timerCallback(const ros::TimerEvent&)
{
	double runTimeDiff = ros::Time::now().toSec()-runTime;

	if(runTimeDiff >=1)
	{
		geometry_msgs::Twist velocities;
		velocities.angular.z = 0;
		velocities.linear.x = 0;
		velPub.publish (velocities);
		runTime =ros::Time::now().toSec();
	}
}

void TrackingControl::reconfigureCallback (robot_navigation::TrackingControlConfig &confg, uint32_t level)
{
	Kp								= confg.Kp;
	Ki								= confg.Ki;
	Kd								= confg.Kd;
	K1								= confg.K1;
	K2								= confg.K2;
	K3								= confg.K3;
	prozentAngVel								= confg.prozentAngVel;
	prozentLinVel								= confg.prozentLinVel;

	robotSpeed						= confg.robotSpeed;
	robotStartStop					= confg.robotStartStop;
}

bool TrackingControl::followObjectCallback(std_srvs::Empty::Request &req,
		std_srvs::Empty::Response &res)
{
	robotStartStop=!robotStartStop;
	return true;
}

void TrackingControl::setRobotSpeed(double speed)
{
	robotSpeed = speed;
}

void TrackingControl::startRobotMovement(bool startMovement)
{

	robotStartStop=startMovement;
	return ;
}

void TrackingControl::odomCallback (const nav_msgs::OdometryConstPtr& odomMsg)
{
	currentAngle=tf::getYaw(odomMsg->pose.pose.orientation);
	currentX=odomMsg->pose.pose.position.x;
	currentY=odomMsg->pose.pose.position.y;
	currentTheta=odomMsg->pose.pose.position.x;
	currentPose.x=currentX;
	currentPose.y=currentY;
	currentPose.yaw=currentTheta;


}
void TrackingControl::startStopCallback (const std_msgs::BoolConstPtr& boolMsg)
{
	robotStartStop = boolMsg->data;
}


