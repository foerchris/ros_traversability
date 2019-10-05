/*
 * robot_navigation.h
 *
 *  Created on: 02.03.2017
 *      Author: getbot
 */

#ifndef ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_ROBOTCONTROLNOPREFIX_H_
#define ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_ROBOTCONTROLNOPREFIX_H_

#include <std_srvs/Empty.h>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <memory>
#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iterator>

#include <std_srvs/Empty.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <dynamixel_msgs/JointState.h>


#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/PointCloud2.h"
#include <message_filters/subscriber.h>
#include "tf/message_filter.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <filters/filter_chain.h>
#include <laser_geometry/laser_geometry.h>
#include <get_std_msgs/IntegerBool.h>

#include <robot_navigation/StepConfig.h>
#include <robot_navigation/TrackingControl.h>
#include "StairBehavior.h"
#include "StepBehavior.h"

class RobotControl
{
	public:
		RobotControl ();
		virtual ~RobotControl ();
		void reconfigureCallback (robot_navigation::StepConfig &confg, uint32_t level);

		bool toggleModeCallback (std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool selectModeCallback (get_std_msgs::IntegerBool::Request &req,
				get_std_msgs::IntegerBool::Response &res);
		bool selectDriveForwardCallback (std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool selectDriveBackwardCallback (std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		void selectFlipperMode(std::size_t mode);
		void currentMode ();
		ros::NodeHandle nodeHandle;
	private:
	    ros::ServiceServer srvSelect;

		enum StateDownwardStep
		{
			RESET, START, SEARCH, DETECTED, MOVE_FORWARD_FLIPPER, MOVE_BACKWARD_FLIPPER, FINISHED
		};

		enum flipperModeEnum
		{
			SNAKE, UPWARD, DOWNWARD ,STAIRSUP, STAIRSDOWN
		};


		void flipperFrontStat_Callback(const dynamixel_msgs::JointStateConstPtr& flipper_state);
		void flipperRearStat_Callback(const dynamixel_msgs::JointStateConstPtr& flipper_state);

		void frontUSS_Callback (const sensor_msgs::RangeConstPtr& laser_front);
		void mid1USS_Callback (const sensor_msgs::RangeConstPtr& laser_mid1);
		void mid2USS_Callback (const sensor_msgs::RangeConstPtr& laser_mid2);
		void rearUSS_Callback (const sensor_msgs::RangeConstPtr& laser_rear);
		std_msgs::Float64 frontFlipperAngleDesiredMsg;
		std_msgs::Float64 rearFlipperAngleDesiredMsg;
		void modeSelectCallback (const std_msgs::BoolConstPtr& sel_mode);
		bool objectSelectedCallback(std_srvs::Empty::Request &req,
				std_srvs::Empty::Response &res);
		void timerCallback(const ros::TimerEvent&);
		void cmdVelCallback (const geometry_msgs::Twist::ConstPtr& cmd_vel);
		//void motorInfoCallback (const drrobot::MotorInfoArray::ConstPtr& motor_info_array);
		void imuCallback (const sensor_msgs::Imu::ConstPtr& imu_ptr);
		void imuCallback2 (const sensor_msgs::Imu::ConstPtr& imu_ptr, VecRPY &currentAngles);
		void laserScanCallback (const sensor_msgs::LaserScan::ConstPtr& laserMsg);
	//	void laserScanCallback (const sensor_msgs::LaserScanConstPtr& laserMsg);
		void HokuyoUTMScanCallback (const sensor_msgs::LaserScanConstPtr& HokulaserMsg);
		void medianFilter (const sensor_msgs::LaserScanConstPtr &laserMsg,  sensor_msgs::LaserScan &flyingPointsFiltered, const int medianFilterRadius, bool threeSixtyDegreeView);

		void FlyingPointsFilterFunction (const sensor_msgs::LaserScanConstPtr &laserMsg, sensor_msgs::LaserScan &flyingPointsFiltered);

		void publishAngles ();
		void publishVelocities();
		void print_Msg ();

		ros::Publisher velPub;
		ros::Publisher frontFlipperAngleDesiredPub;
		ros::Publisher rearFlipperAngleDesiredPub;
		ros::Publisher velocityLimitationPub;
		ros::Publisher laseranglePub;
		ros::Publisher HokuanglePub;
		bool startStairBehavior;

		std_msgs::Bool velocityLimitationMsg;
		//sensor_msgs::PointCloud2 cloud;

		ros::Subscriber flipperFrontStateSub;
		ros::Subscriber flipperRearStateSub;

		ros::Subscriber frontUSS_Sub;
		ros::Subscriber mid1USS_Sub;
		ros::Subscriber mid2USS_Sub;
		ros::Subscriber rearUSS_Sub;
		ros::Subscriber cmdVelSub;
		ros::Subscriber imuSub;
		ros::Subscriber laserScanSub;
		ros::Subscriber HokuyoUTMScanSub;
		ros::Subscriber motorInfoSub;
		ros::Subscriber modeSelectSub;

		ros::ServiceServer toggleModeService;
		ros::ServiceServer selectModeService;

		ros::ServiceServer selectDriveForwardService;
		ros::ServiceServer selectDriveBackwardService;

		std::string BASE_FRAME = "GETjag/base_link";
		std::string BASE_FRAME_ODOM =  "GETjag/odom";
		//std::string BASE_FRAME = "GETjag/ground";
		std::unique_ptr<tf::TransformListener> tfListener;
		std::unique_ptr<laser_geometry::LaserProjection> projector;
		DistanceVec front;
		DistanceVec rear;
		DistanceVec front_rotBodyAngle;
		DistanceVec rear_rotBodyAngle;
		DistanceVec front_horizontal;

		TrackingControl* trackingControl = new TrackingControl(nodeHandle, 0.5, 0.2, 0.05);

		ros::Timer timer;
		double runTime;
		double currentTime;
		double previousTime;
		double rear_angle_max;
		double rear_angle_min;
		double front_angle_max;
		double front_angle_min;
		double mid1_uss;
		double mid2_uss;
		double front_uss;
		double rear_uss;

		double currentFlipperAngleFront;
		double currentFlipperAngleRear;
		double R;		// Radius of the wheel of the robot body
		double r;		// Radius of the wheel on the end of the flipper
		double L;		// Wheel distance between the robot axes
		double l;		// Wheel distance between the robot wheel axis and the wheel axis on the end of the flipper
		double w;		// Distance between center to the outer flipper
		double d;		// Distance between body wheel axis and "downside of flipper wheel"
		double theta;	// Angle of distance (see above) and flipper wheel radius
		//double S;		// Distance from end of robot body to front wheel axis
		double s;		// Distance from the middle to the end of robot body
		double base_offset;

		double frontDesiredAngle;
		double rearDesiredAngle;

		double lastFrontDesiredAngle;
		double lastRearDesiredAngle;
		double stepBeta;
		double stepZ;

		double frontFlipperCurrent;
		double rearFlipperCurrent;
		double bodyAngle;
		double bodyAngleRoll;
		double bodyAnglePitch;
		double linearVel;
		double angularVel;
		double angleThreshold;
		int flipperMode;
		int flipperModeOld;
		int robotDirection;
		int stateStep;

		bool velocityLimitation;
		bool velocityLimitationOld;

		bool laserAvailable;
		bool HokuAvailable;
		bool imuAvailable;

		double _Jumpthreshold = 0.10;
		double _ObstacleSize = 0.05;
		double _angleThreshold = 0.2;
		double _NeighbourDifferenceThreshold = 0.03;
		int _nextComparedIndex = 1;

		// Dynamic reconfigure variables
		double body_front_0_distance_offset;
		double body_front_1_distance_offset;
		double body_rear_0_distance_offset;
		double body_rear_1_distance_offset;

		double snake_mode_front_angle_min;
		double snake_mode_rear_angle_min;


		double step_height_max;

		double flipper_angle_min;
		double flipper_angle_max;

		double laser_scan_front_angle_min;
		double laser_scan_front_angle_max;
		double laser_scan_rear_angle_min;
		double laser_scan_rear_angle_max;
		StairBehavior stairBehavior;
		SnakeMode snakeModeObject;
		StepBehavior stepBehavior;
		double Hoku_scan_angle_min;
		double Hoku_scan_angle_max;
		bool selectMode;
		bool selectDriveForward;
		bool selectDriveBackward;
		bool moveRobot;
};

#endif /* ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_ROBOTCONTROLNOPREFIX_H_ */
