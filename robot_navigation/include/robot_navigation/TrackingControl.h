/*
 * TrackingControl.h
 *
 *  Created on: Oct 5, 2017
 *      Author: chfo
 */

#ifndef SRC_ROBOT_NAVIGATION_FOR_3D_STRUCTURES_SRC_TRACKINGCONTROL_H_
#define SRC_ROBOT_NAVIGATION_FOR_3D_STRUCTURES_SRC_TRACKINGCONTROL_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

#include <ctime>

#include <dynamic_reconfigure/server.h>
#include <robot_navigation/TrackingControlConfig.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_srvs/Empty.h>

struct pose {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
} ;
// this class provides methods to calculate velocities depending on a goal pos and publish this velocities
class TrackingControl {
public:
	TrackingControl (ros::NodeHandle nh, double turn_speed, double fast_speed, double init_speed);
	virtual ~TrackingControl ();
	// drive to a point given by its x,y and z koordinates in a certain pos
	bool pointToPointCalc(const std::vector<pose>& globalCoordinates, bool reset);
    // following a couture by controlling the y deviation and setting the rotational velocity
  	void simpleYDeviationControl(double xGoal, double yGoal , double zGoal, bool lineFound);

  	void simpleThetaDeviationControl(double xGoal, double yGoal , double zGoal,bool lineFound);

    // following a couture by controlling the y deviation and setting the rotational velocity + behavior for the case of a lost couture
	void yDeviationControl(std::vector<pose> coordinates,bool lineFound);
    // following a couture by controlling the theta deviation and setting the rotational velocity + behavior for the case of a lost couture
	void thetaDeviationControl(std::vector<pose> coordinates,bool lineFound);
    // holds the xtion camera in a predefined position 1 for forward and -1 for backward
	void alginCamera(const bool& choseFrontRearView);
	void setOriantation(const int& choseForBackMode);
	// follow a reference path
	bool referencePath(const std::vector<pose>& coordinates, bool reset);
	// drive to a point given by x,y and z with the tangential escape method
	void tangentialEscape(double xGoal, double yGoal , double zGoal,bool lineFound);
	// approach a point given by its x,y and z coordinates by using Odometrie data
	bool approachLine(std::vector<pose> coordinates, bool lineFound);
	void robotToGlobaleTransform(pose &Pose);
	void setLinearVel(double vel);
	void setDirection(const int& direc);
	int sgn(double x);
	void startRobotMovement(bool startMovement);
	bool pointDistanzCheak(const pose &globalCoordinates);

	void setRobotSpeed(double speed);
	void followPathCallback(const ros::TimerEvent& bla);
	void setPath(std::vector<pose> path);
	void setFollowPath(bool follow);
private:
	void reconfigureCallback (robot_navigation::TrackingControlConfig &confg, uint32_t level);
	bool followObjectCallback(std_srvs::Empty::Request &req,
				std_srvs::Empty::Response &res);
	void globaleToRobotTransform(pose &Pose);
	double calcTheta(pose P1, pose P2);
	bool compareCord(std::vector<double> P1, std::vector<double> P2);
	void timerCallback(const ros::TimerEvent&);
	void odomCallback (const nav_msgs::OdometryConstPtr& odomMsg);
	bool comparAngle(double difAngle, double refAngle);
	double umwanldung720(double wink);
	double gefahrendestrecke(double streckex,double streckey,double dsx,double dsy,double x_0,double y_0);
	void getCurrentRobotPose (std::string odom_frame, std::string robot_frame, geometry_msgs::PoseStamped& pose);
	double distanceTo(pose p1, pose p2);
	double clcCurvature(std::vector<pose> curvPoints);
	double area2(pose p1, pose p2 , pose p3);
	void panCamera();
	void startStopCallback (const std_msgs::BoolConstPtr& boolMsg);


	ros::Publisher markerPublisher;
	visualization_msgs::Marker createMarker (std::string ns, int id, double x, double y,  double r, double g , double b , double a );
	// ROS publisher
	ros::Publisher velPub;
	geometry_msgs::Twist velocities;
	ros::Publisher 	camAnglePitchPub;
	ros::Publisher 	camAngleYawPub;
	ros::Subscriber	odomSub;
	ros::Subscriber robotStartStopSub;

	ros::Timer followPathTimer;
	std::vector<pose> globalPath;
	bool followPath;
	std_msgs::Float64 camAnglePitch;
	std_msgs::Float64 camAngleYaw;

	// ROS Services
    ros::ServiceServer srvFollow;
	int forwardMode;

	int direction;
	std::string tf_prefix;
	std::string BASE_FRAME;
	std::string MAP_FRAME;
	std::string ODOM_FRAME;


	std::unique_ptr<tf::TransformListener> tfListener;
	tf::StampedTransform transform;

	enum State
	{
		START, ROTATE_GOAL, MOVE_FOR, ROTATE_ORIENTATION
	};
	State currentTrackingState;


	double dsx,dsy;
	double dAngle;
	int save;
	int count;
	double lastTheta;
	enum LineFollowStat
	{
			start,followLine, rotateLine
	};
	LineFollowStat lineFollowStat;

	//ROS timer
	ros::Timer timer;

	// dynmaic reconfigure
	pose currentPose;
	double runTime;
	double currentAngle;
	double currentX;
	double currentY;
	double currentTheta;
	bool robotStartStop;
	double robotSpeed;
	double Kp;
	double Ki;
	double Kd;
	double K1;
	double K2;
	double K3;
	double prozentAngVel;
	double prozentLinVel;
	double currentTime;
	double previousTime;
	double MAX_LINEAR_VEL = 1;
	double MAX_ANGULAR_VEL = 2;

};




#endif /* SRC_ROBOT_NAVIGATION_FOR_3D_STRUCTURES_SRC_TRACKINGCONTROL_H_ */
