#include "robot_navigation/RobotControl.h"

constexpr double D2R = M_PI / 180.0;
constexpr double R2D = 180.0 / M_PI;


constexpr double DEGREE_20 = 80.0;
constexpr double DEGREE_110  = 440.0;
constexpr double DEGREE_200 = 800.0;

RobotControl::RobotControl ()
{
	srvSelect = nodeHandle.advertiseService("select_stair_Search", &RobotControl::objectSelectedCallback, this);

	startStairBehavior=false;
	currentTime = ros::Time::now ().toSec ();
	previousTime = currentTime;
	R = 0.08;
	r = 0.0375;
	L = 0.48;
	l = 0.22;
	w = 0.26;
	d = std::hypot (r, l);
	theta = atan (r / l);
//	S = L + R;
	s = (L / 2.0) + R;


	linearVel = 0.0;
	angularVel = 0.0;
	stepBeta = 0.0;
	stepZ = 0.0;
	imuAvailable = false;
	laserAvailable = false;
	velocityLimitationOld = false;
	velocityLimitation = false;
	flipperMode = 0;
	flipperModeOld = flipperMode;
	robotDirection = 0;
	stateStep = -1;
	angleThreshold = -8.0;
	selectMode = false;
	selectDriveForward = true;
	selectDriveBackward = false;
	rear_angle_min = -90.0;
	rear_angle_max = 90.0;
	front_angle_min = -90.0;
	front_angle_max = 90.0;
	lastFrontDesiredAngle = 0.0;
	lastRearDesiredAngle = 0.0;
	moveRobot = true;


	/* // Dynamic reconfigure values
	 step_mode_front_angle_min = -45.0;
	 snake_mode_front_angle_min = -45.0;
	 snake_mode_rear_angle_min = -45.0;
	 step_rear_height_treshold = 0.04;
	 body_angle_treshold = 20.0;
	 step_offset = 0.02;
	 body_front_0_distance_offset = 0.0;
	 body_front_1_distance_offset = 0.0;
	 body_rear_0_distance_offset = 0.0;
	 body_rear_1_distance_offset = 0.0;
	 step_angle_treshold = 0.5;
	 step_height_treshold = 0.15;
	 step_distance_treshold_min = 0.18;
	 step_distance_treshold_max = 0.3;
	 flipper_front_angle_step_climb = 130.0;
	 flipper_angle_min = 20.0;
	 flipper_angle_max = 150.0;
	 laser_scan_front_angle_min = 48.0 * D2R;
	 laser_scan_front_angle_max = 68.0 * D2R;
	 laser_scan_rear_angle_min = -21.0 * D2R;
	 laser_scan_rear_angle_max = -58.0 * D2R;
	 Hoku_scan_angle_min = -27.0 * D2R;
	 Hoku_scan_angle_max = 15.0 * D2R;snakeMode
	 */

	static dynamic_reconfigure::Server<robot_navigation::StepConfig> server (ros::NodeHandle ("~/RobotControl"));
	static dynamic_reconfigure::Server<robot_navigation::StepConfig>::CallbackType f;
	f = boost::bind (&RobotControl::reconfigureCallback, this, _1, _2);
	server.setCallback (f);

	toggleModeService = nodeHandle.advertiseService ("flipper_control_toogle_mode", &RobotControl::toggleModeCallback, this);

	selectModeService = nodeHandle.advertiseService ("flipper_control_select_mode", &RobotControl::selectModeCallback, this);

	selectDriveForwardService = nodeHandle.advertiseService ("motor_control_select_drive_forward", &RobotControl::selectDriveForwardCallback, this);

	selectDriveBackwardService = nodeHandle.advertiseService ("motor_control_select_drive_backward", &RobotControl::selectDriveBackwardCallback, this);

	//Publisher
	velPub = nodeHandle.advertise < geometry_msgs::Twist > ("cmd_vel", 1);
	laseranglePub = nodeHandle.advertise < sensor_msgs::LaserScan > ("laser_angles", 1);
	HokuanglePub = nodeHandle.advertise < sensor_msgs::LaserScan > ("hoku_angles", 1);

	//Subscriber
	frontUSS_Sub = nodeHandle.subscribe < sensor_msgs::Range > ("ground_clearance_1", 1, boost::bind (&RobotControl::frontUSS_Callback, this, _1));
	mid1USS_Sub = nodeHandle.subscribe < sensor_msgs::Range > ("ground_clearance_2", 1, boost::bind (&RobotControl::mid1USS_Callback, this, _1));
	mid2USS_Sub = nodeHandle.subscribe < sensor_msgs::Range > ("ground_clearance_3", 1, boost::bind (&RobotControl::mid2USS_Callback, this, _1));
	rearUSS_Sub = nodeHandle.subscribe < sensor_msgs::Range > ("ground_clearance_4", 1, boost::bind (&RobotControl::rearUSS_Callback, this, _1));

	flipperFrontStateSub = nodeHandle.subscribe < dynamixel_msgs::JointState > ("flipper_front_controller/state", 1, boost::bind (&RobotControl::flipperFrontStat_Callback, this, _1));
	flipperRearStateSub = nodeHandle.subscribe < dynamixel_msgs::JointState > ("flipper_rear_controller/state", 1, boost::bind (&RobotControl::flipperRearStat_Callback, this, _1));

	cmdVelSub = nodeHandle.subscribe < geometry_msgs::Twist > ("cmd_vel", 1, boost::bind (&RobotControl::cmdVelCallback, this, _1));

	HokuyoUTMScanSub = nodeHandle.subscribe < sensor_msgs::LaserScan > ("laser_scan_front", 1, boost::bind (&RobotControl::HokuyoUTMScanCallback, this, _1));
	laserScanSub = nodeHandle.subscribe < sensor_msgs::LaserScan > ("laser_scan_mid", 1, boost::bind (&RobotControl::laserScanCallback, this, _1));

	frontFlipperAngleDesiredPub = nodeHandle.advertise < std_msgs::Float64 > ("flipper_front_controller/command", 1);
	rearFlipperAngleDesiredPub = nodeHandle.advertise < std_msgs::Float64 > ("flipper_rear_controller/command", 1);

	imuSub = nodeHandle.subscribe < sensor_msgs::Imu > ("imu/data", 1, boost::bind (&RobotControl::imuCallback, this, _1));

	// Transform listener
	tfListener = std::unique_ptr < tf::TransformListener > (new tf::TransformListener);
}
RobotControl::~RobotControl ()
{
}

void RobotControl::flipperFrontStat_Callback(const dynamixel_msgs::JointStateConstPtr& flipper_state)
{
	currentFlipperAngleFront = flipper_state->current_pos;
}

void RobotControl::flipperRearStat_Callback(const dynamixel_msgs::JointStateConstPtr& flipper_state)
{
	currentFlipperAngleRear = flipper_state->current_pos;
}


void RobotControl::modeSelectCallback (const std_msgs::BoolConstPtr& sel_mode)
{

	ROS_INFO ("Changed MODE!!!!!!");
	if (sel_mode->data)
	{
		if (flipperMode < 3)
			flipperMode++;
		else
			flipperMode = 0;

		if (flipperMode != 1)
			velocityLimitation = false;
	}
}
void RobotControl::frontUSS_Callback (const sensor_msgs::RangeConstPtr& laser_front)
{
	front.vd = laser_front->range;
	front_uss = laser_front->range;

}
/**
 * Gets the values of the first mid ultrasonic sensor.
 * @param laser_mid1
 */
void RobotControl::mid1USS_Callback (const sensor_msgs::RangeConstPtr& laser_mid1)
{
	mid1_uss = laser_mid1->range;
}

/**
 * Gets the vaules of the second mid ultrasonic sensor.
 * @param laser_mid2
 */
void RobotControl::mid2USS_Callback (const sensor_msgs::RangeConstPtr& laser_mid2)
{
	mid2_uss = laser_mid2->range;
}

/**
 * Gets the values of the rear ultrasonic sensor.
 * @param laser_rear
 */
void RobotControl::rearUSS_Callback (const sensor_msgs::RangeConstPtr& laser_rear)
{

	rear.vd = laser_rear->range;
	rear_uss = laser_rear->range;
	ROS_INFO ("******************* Ultrasound front: [%7.3f] \t mid1: [%7.3f] ", rear.vd,rear_uss);
}

/**
 * Gets the cmd values.
 * @param cmd_vel
 */
void RobotControl::cmdVelCallback (const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	linearVel = cmd_vel->linear.x;
	angularVel = cmd_vel->angular.z;
}

/**
 * Gets the imu values
 * @param imu_ptr
 */
void RobotControl::imuCallback (const sensor_msgs::Imu::ConstPtr& imu_ptr)
{
	double x = imu_ptr->orientation.x;
	double y = imu_ptr->orientation.y;
	double z = imu_ptr->orientation.z;
	double w = imu_ptr->orientation.w;
	double roll;
	double pitch;
	double yaw;

	tf::Quaternion q (x, y, z, w);
	tf::Matrix3x3 m (q);
	m.getRPY (roll, pitch, yaw);
	bodyAnglePitch = pitch;
	bodyAngleRoll = roll;
	bodyAngle = (round (pitch* R2D * 10.0)) / 10.0;
	stairBehavior.SetBodyAngle (bodyAnglePitch, bodyAngleRoll, bodyAngle);
	stepBehavior.SetBodyAngle (bodyAnglePitch, bodyAngleRoll, bodyAngle);

	imuAvailable = true;
}

/**
 * Gets the values of the laser scanner.
 * @param laserMsg
 */

void RobotControl::selectFlipperMode(std::size_t mode)
{
	if(mode<6 && mode>=0)
	{
		flipperMode=mode;
		if (flipperMode != 1)
		{
				velocityLimitation = false;
		}
	}
}


void RobotControl::laserScanCallback (const sensor_msgs::LaserScan::ConstPtr& laserMsg)
{
	int medianWindowSize = 2;
	sensor_msgs::LaserScan flyingPointsFiltered;
	sensor_msgs::LaserScan laser_angle;
	laser_angle = *laserMsg;
	std::cout<<"flipperMode"<< flipperMode<< std::endl;
	// Flying point filter
	flyingPointsFiltered.header = laserMsg->header;
	flyingPointsFiltered.angle_min = laserMsg->angle_min;
	flyingPointsFiltered.angle_max = laserMsg->angle_max;
	flyingPointsFiltered.range_min = laserMsg->range_min;
	flyingPointsFiltered.range_max = laserMsg->range_max;
	flyingPointsFiltered.angle_increment = laserMsg->angle_increment;
	flyingPointsFiltered.time_increment = laserMsg->time_increment;
	flyingPointsFiltered.scan_time = laserMsg->scan_time;
	medianFilter(laserMsg,flyingPointsFiltered,medianWindowSize,false);
	FlyingPointsFilterFunction (laserMsg, flyingPointsFiltered);
	const std::string& scanFrameID = flyingPointsFiltered.header.frame_id;
	const ros::Time& scanTimeStamp = ros::Time(0);//flyingPointsFiltered.header.stamp;
	sensor_msgs::PointCloud2 cloud;
	try
	{
		tfListener->waitForTransform (BASE_FRAME, scanFrameID, scanTimeStamp, ros::Duration (3.0));

	}
	catch (tf::ExtrapolationException& e)
	{
		ROS_ERROR ("laserScan %s", e.what ());
		return;
	}

	double angle = flyingPointsFiltered.angle_min;
	front.z.clear ();
	front.x.clear ();
	rear.z.clear ();
	rear.x.clear ();
	front_rotBodyAngle.x.clear ();
	front_rotBodyAngle.z.clear ();
	rear_rotBodyAngle.x.clear ();
	rear_rotBodyAngle.z.clear ();

	bool nan;
	// Filter the values if not in a certain angle
	for (std::size_t i = 0; i < laser_angle.ranges.size (); i++)
	{
		nan = true;
		if ((angle >= laser_scan_front_angle_min) and (angle <= laser_scan_front_angle_max))
		{
			nan = false;
		}
		if ((angle <= laser_scan_rear_angle_min) and (angle >= laser_scan_rear_angle_max))
			nan = false;

		if (nan)
			laser_angle.ranges.at (i) = 0.1;
		angle += flyingPointsFiltered.angle_increment;
	}

	angle = flyingPointsFiltered.angle_min;
	// Filter the values if not in a certain range
	for (double range : flyingPointsFiltered.ranges)
	{

		// Transformation for front values
		if ((angle >= laser_scan_front_angle_min) and (angle <= laser_scan_front_angle_max))
		{
			/*if ((not std::isnormal (range)) || (range <= flyingPointsFiltered.range_min) || (range >= flyingPointsFiltered.range_max))
				range = flyingPointsFiltered.range_max;*/

			if ((std::isnormal (range)) && (range > flyingPointsFiltered.range_min) && (range < flyingPointsFiltered.range_max))
			{

				double xx = cos (angle - ((bodyAngle) * D2R)) * range;
				double yy = -sin (angle - ((bodyAngle) * D2R)) * range;
				double x = cos (angle) * range;
				double y = sin (angle) * range;

	//			std::cout<<"range: "<<range<<" x: "<<x<<" y: "<<y<<std::endl;
	//			printf ("range: %4.4f  x: %4.4f  y: %4.4f \n", range, x, y);
				tf::Stamped < tf::Point > point (tf::Point (x, y, 0), scanTimeStamp, scanFrameID);
				tf::Stamped < tf::Point > point2 (tf::Point (xx, yy, 0), scanTimeStamp, scanFrameID);
				tf::Stamped < tf::Point > pointTransformed;
				tf::Stamped < tf::Point > pointTransformed2;
				try
				{
					tfListener->transformPoint (BASE_FRAME, scanTimeStamp, point, scanFrameID, pointTransformed);
					tfListener->transformPoint (BASE_FRAME, scanTimeStamp, point2, scanFrameID, pointTransformed2);

				}
				catch (tf::TransformException& ex)
				{
					ROS_ERROR ("laserFilter (front) %s", ex.what ());
					return;
				}
				front_rotBodyAngle.x.push_back (pointTransformed2.getX () - s);
				front_rotBodyAngle.z.push_back (pointTransformed2.getZ () + base_offset);

				front.x.push_back (pointTransformed.getX () - s);
				front.z.push_back (pointTransformed.getZ () + base_offset);
			}
		}
		// Transformation for rear values
		if ((angle <= laser_scan_rear_angle_min) and (angle >= laser_scan_rear_angle_max) and !(std::isnan (range)))
		{
			/*if ((not std::isnormal (range)) || (range <= flyingPointsFiltered.range_min) || (range >= flyingPointsFiltered.range_max))
			{
				range = flyingPointsFiltered.range_max; //
			}*/

			if ((std::isnormal (range)) && (range > flyingPointsFiltered.range_min) && (range < flyingPointsFiltered.range_max))
			{
				double xx = cos (angle - ((bodyAngle) * D2R)) * range;
				double yy = -sin (angle - ((bodyAngle) * D2R)) * range;
				double x = cos (angle) * range;
				double y = sin (angle) * range;
				tf::Stamped < tf::Point > point (tf::Point (x, y, 0), scanTimeStamp, scanFrameID);
				tf::Stamped < tf::Point > point2 (tf::Point (xx, yy, 0), scanTimeStamp, scanFrameID);
				tf::Stamped < tf::Point > pointTransformed;
				tf::Stamped < tf::Point > pointTransformed2;
				try
				{
					tfListener->transformPoint (BASE_FRAME, scanTimeStamp, point, scanFrameID, pointTransformed);
					tfListener->transformPoint (BASE_FRAME, scanTimeStamp, point2, scanFrameID, pointTransformed2);
				}
				catch (tf::TransformException& ex)
				{
					ROS_ERROR ("laserFilter (rear) %s", ex.what ());
					return;
				}
				rear_rotBodyAngle.x.insert (rear_rotBodyAngle.x.begin (), pointTransformed2.getX () + s);
				rear_rotBodyAngle.z.insert (rear_rotBodyAngle.z.begin (), pointTransformed2.getZ () + base_offset);
				rear.x.insert (rear.x.begin (), pointTransformed.getX () + s);
				rear.z.insert (rear.z.begin (), pointTransformed.getZ () + base_offset);
			}
		}
		angle += flyingPointsFiltered.angle_increment;
	}
	laserAvailable = true;
	currentMode();
	laseranglePub.publish (laser_angle);
}
/**
 * Gets the values of the hokuyou scanner
 * @param HokulaserMsg
 */
void RobotControl::HokuyoUTMScanCallback (const sensor_msgs::LaserScanConstPtr& HokulaserMsg)
{

	int medianWindowSize = 2;
	sensor_msgs::LaserScan flyingPointsFiltered;
	sensor_msgs::LaserScan Hoku_angle;

	Hoku_angle = *HokulaserMsg;
	medianFilter(HokulaserMsg,Hoku_angle,medianWindowSize,false);

	flyingPointsFiltered.header = HokulaserMsg->header;
	flyingPointsFiltered.angle_min = HokulaserMsg->angle_min;
	flyingPointsFiltered.angle_max = HokulaserMsg->angle_max;
	flyingPointsFiltered.range_min = HokulaserMsg->range_min;
	flyingPointsFiltered.range_max = HokulaserMsg->range_max;
	flyingPointsFiltered.angle_increment = HokulaserMsg->angle_increment;
	flyingPointsFiltered.time_increment = HokulaserMsg->time_increment;
	flyingPointsFiltered.scan_time = HokulaserMsg->scan_time;

	const std::string& scanFrameID = flyingPointsFiltered.header.frame_id;
	const ros::Time& scanTimeStamp = ros::Time(0);//flyingPointsFiltered.header.stamp;
	try
	{
		tfListener->waitForTransform (BASE_FRAME, scanFrameID, scanTimeStamp, ros::Duration (2.0));
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR ("HokuyoUTMScane %s", ex.what ());
		return;
	}
	double angle = flyingPointsFiltered.angle_min;

	front_horizontal.x.clear ();
	front_horizontal.y.clear ();
	front_horizontal.z.clear ();

	bool nan;
	for (std::size_t i = 0; i < Hoku_angle.ranges.size (); i++)
	{
		nan = true;

		if ((angle >= Hoku_scan_angle_min) and (angle <= Hoku_scan_angle_max))
			nan = false;

		if (nan)
			Hoku_angle.ranges.at (i) = 0.1;

		angle += Hoku_angle.angle_increment;
	}

	angle = Hoku_angle.angle_min;
	for (double range : Hoku_angle.ranges)
	{

		if ((angle >= Hoku_scan_angle_min) and (angle <= Hoku_scan_angle_max) and !(std::isnan (range)))
		{

			//if ((not std::isnormal (range)) || (range <= Hoku_angle.range_min) || (range >= Hoku_angle.range_max))
			//	range = Hoku_angle.range_max;

			if ((std::isnormal (range)) && (range > Hoku_angle.range_min) && (range < Hoku_angle.range_max))
			{
				double x = cos (angle) * range;
				double y = sin (angle) * range;

				tf::Stamped < tf::Point > point (tf::Point (x, y, 0), scanTimeStamp, scanFrameID);
				tf::Stamped < tf::Point > pointTransformed;

				try
				{
					tfListener->transformPoint (BASE_FRAME, scanTimeStamp, point, scanFrameID, pointTransformed);
				}
				catch (tf::TransformException& ex)
				{
					ROS_ERROR ("laserFilter (front) %s", ex.what ());
					return;
				}

				front_horizontal.x.push_back (pointTransformed.getX () - s);
				front_horizontal.y.push_back (pointTransformed.getY () + w);
				front_horizontal.z.push_back (pointTransformed.getZ ());
			}
		}
		angle += Hoku_angle.angle_increment;
	}
	HokuAvailable = true;
	HokuanglePub.publish (Hoku_angle);
}

void RobotControl::medianFilter (const sensor_msgs::LaserScanConstPtr &laserMsg,  sensor_msgs::LaserScan &flyingPointsFiltered, const int medianFilterRadius, bool threeSixtyDegreeView)
{
	if (medianFilterRadius < 1)
	{	throw "filter radius < 1";
	}
	sensor_msgs::LaserScan laser_angle = *laserMsg;

	const int elementCount = medianFilterRadius * 2 + 1;

	double elements [elementCount];

	std::size_t range = laser_angle.ranges.size();
	for (std::size_t i = 0; i < range; i++)
	{
		std::size_t j = 0;
		for (std::size_t index = i - medianFilterRadius; index <= i + medianFilterRadius; index++)
		{
			int idx = index;
			if(threeSixtyDegreeView)
			{
				idx=(range-index)%range;
			}
			else
			{
				if(index < 0)
				{
					idx=index+medianFilterRadius;
				}
				else if(index >= range)
				{
					idx=index-medianFilterRadius;
				}
			}
			elements [j++] = laser_angle.ranges.at(idx);
		}

		std::sort (elements, elements + elementCount);

		laser_angle.ranges.at(i)=elements [medianFilterRadius];
	}
	flyingPointsFiltered = laser_angle;
}
/**
 * Flying point filter for filtering the laser scanner data
 * @param laserMsg
 * @param flyingPointsFiltered
 */
void RobotControl::FlyingPointsFilterFunction (const sensor_msgs::LaserScanConstPtr &laserMsg, sensor_msgs::LaserScan &flyingPointsFiltered)
{
	trackingControl->setLinearVel(0.1);
	sensor_msgs::LaserScan flyingPointsFilteredTemp;
	int _nextComparedIndex = 1;
	double _angleThreshold = 0.2;
	double _NeighbourDifferenceThreshold = 0.03;

	flyingPointsFilteredTemp.header = laserMsg->header;
	flyingPointsFilteredTemp.angle_min = laserMsg->angle_min;
	flyingPointsFilteredTemp.angle_max = laserMsg->angle_max;
	flyingPointsFilteredTemp.range_min = laserMsg->range_min;
	flyingPointsFilteredTemp.range_max = laserMsg->range_max;
	flyingPointsFilteredTemp.angle_increment = laserMsg->angle_increment;
	flyingPointsFilteredTemp.time_increment = laserMsg->time_increment;
	flyingPointsFilteredTemp.scan_time = laserMsg->scan_time;

	const uint32_t ranges_size = laserMsg->ranges.size ();
	flyingPointsFilteredTemp.ranges.assign (ranges_size, flyingPointsFilteredTemp.range_max);


	float x_i, y_i, x_i_1, y_i_1;
	float x_diff, y_diff, NeighbouringPointsDifference;
	float angleTemp_i, angleTemp_i_1, x_y_angle;
	bool Filtered[ranges_size];

	double angle = flyingPointsFilteredTemp.angle_min;

	int rangeSize = ranges_size;

	for (int i = 0; i < rangeSize; i++)
	{
		if (((angle >= laser_scan_front_angle_min) and (angle <= laser_scan_front_angle_max)) or ((angle <= laser_scan_rear_angle_min) and (angle >= laser_scan_rear_angle_max)))
		{
			angleTemp_i = 0;
			angleTemp_i_1 = 0;
			x_y_angle = 0;

			if (i * laserMsg->angle_increment <= laserMsg->angle_increment * DEGREE_20)
			{
				angleTemp_i = -(laserMsg->angle_increment * DEGREE_20 - (i * laserMsg->angle_increment));
			}
			else if ((i * laserMsg->angle_increment > laserMsg->angle_increment * DEGREE_20) && (i * laserMsg->angle_increment <= laserMsg->angle_increment * DEGREE_110))
			{
				angleTemp_i = (i * laserMsg->angle_increment) - (laserMsg->angle_increment * DEGREE_20);
			}
			else if ((i * laserMsg->angle_increment > laserMsg->angle_increment * DEGREE_110) && (i * laserMsg->angle_increment <= laserMsg->angle_increment * DEGREE_200))
			{
				angleTemp_i = (laserMsg->angle_increment * DEGREE_200) - (i * laserMsg->angle_increment);
			}
			else
				angleTemp_i = -((i * laserMsg->angle_increment) - (laserMsg->angle_increment * DEGREE_200));

			if ((i + _nextComparedIndex) * laserMsg->angle_increment <= laserMsg->angle_increment * DEGREE_20)
			{
				angleTemp_i_1 = -(laserMsg->angle_increment * DEGREE_20 - ((i + _nextComparedIndex) * laserMsg->angle_increment));
			}
			else if (((i + _nextComparedIndex) * laserMsg->angle_increment > laserMsg->angle_increment * DEGREE_20) && ((i + _nextComparedIndex) * laserMsg->angle_increment <= laserMsg->angle_increment * DEGREE_110))
			{
				angleTemp_i_1 = ((i + _nextComparedIndex) * laserMsg->angle_increment) - (laserMsg->angle_increment * DEGREE_20);
			}
			else if (((i + _nextComparedIndex) * laserMsg->angle_increment > laserMsg->angle_increment * DEGREE_110) && ((i + _nextComparedIndex) * laserMsg->angle_increment <= laserMsg->angle_increment * DEGREE_200))
			{
				angleTemp_i_1 = (laserMsg->angle_increment * DEGREE_200) - ((i + _nextComparedIndex) * laserMsg->angle_increment);
			}
			else
				angleTemp_i_1 = -(((i + _nextComparedIndex) * laserMsg->angle_increment) - (laserMsg->angle_increment * DEGREE_200));

			x_i = cos (angleTemp_i) * laserMsg->ranges[i];
			y_i = sin (angleTemp_i) * laserMsg->ranges[i];

			x_i_1 = cos (angleTemp_i_1) * laserMsg->ranges[i + _nextComparedIndex];
			y_i_1 = sin (angleTemp_i_1) * laserMsg->ranges[i + _nextComparedIndex];

			x_diff = x_i_1 - x_i;
			y_diff = y_i_1 - y_i;

			NeighbouringPointsDifference = sqrtf (powf (x_diff, 2) + powf (y_diff, 2));
			x_y_angle = atanf (y_diff / x_diff);

			if ((fabs (x_y_angle - angleTemp_i) > _angleThreshold && NeighbouringPointsDifference <= _NeighbourDifferenceThreshold) || laserMsg->ranges[i] >= 3)
			{
				flyingPointsFilteredTemp.ranges[i] = laserMsg->ranges[i];
				Filtered[i] = false;
			}
			else
			{
				Filtered[i] = true;
			}
		}

		angle += flyingPointsFilteredTemp.angle_increment;
	}

	angle = flyingPointsFilteredTemp.angle_min;

	for (int i = 1; i < rangeSize - 3; i++)
	{
		if (((angle >= laser_scan_front_angle_min) and (angle <= laser_scan_front_angle_max)) or ((angle <= laser_scan_rear_angle_min) and (angle >= laser_scan_rear_angle_max)))
		{
			// Filters points which are less than 10cm away from laser and outlier with a distance of 2 cm to each other
			if (flyingPointsFilteredTemp.ranges[i] <= 0.10 || (fabs (flyingPointsFilteredTemp.ranges[i] - flyingPointsFilteredTemp.ranges[i + 1]) >= 0.02))
			{
				flyingPointsFilteredTemp.ranges[i] = NAN;
				Filtered[i] = true;
			}

			if (Filtered[i - 1] == false && Filtered[i] == true)
			{
				if (Filtered[i + 1] == false || Filtered[i + 2] == false)
				{ //check next 2 indices if one is not filtered => If so, i should not be filtered too
					Filtered[i] = false;
					flyingPointsFilteredTemp.ranges[i] = laserMsg->ranges[i];

				}
			}

			if (Filtered[i - 1] == true && Filtered[i] == false)
			{
				if (Filtered[i + 1] == true || Filtered[i + 2] == true)
				{
					Filtered[i] = true;
					flyingPointsFilteredTemp.ranges[i] = NAN;
				}
			}
		}
		angle += flyingPointsFilteredTemp.angle_increment;
	}

	flyingPointsFiltered = flyingPointsFilteredTemp;

}

/**
 * Main function
 */
void RobotControl::currentMode ()
{
	bool pubAng = false;
	bool rotate = false;
	moveRobot = true;
	if (flipperMode != flipperModeOld)
	{
		robotDirection = 0;
		stepBehavior.stepReset ();
	}
	if (angularVel != 0.0 and linearVel == 0.0)
	{
		double phi = 90.0;
		frontDesiredAngle = phi;
		rearDesiredAngle = phi;
		robotDirection = 0;
		stepBehavior.stepReset ();
		pubAng = true;
		rotate = true;

	}
	else
	{

		// Robot moves forward
		if ((linearVel > 0.0) or ((linearVel == 0.0) and (robotDirection > 0)))
		{
			if (robotDirection < 0)
				stepBehavior.stepReset ();

			robotDirection = 1;
		}
		// Robot moves backwards
		else if ((linearVel < 0.0) or ((linearVel == 0.0) and (robotDirection < 0)))
		{
			if (robotDirection > 0)
				stepBehavior.stepReset ();

			robotDirection = -1;
		}
		// Robot stops
		else if ((linearVel == 0.0) and (robotDirection == 0))
		{
			robotDirection = 0;
		}
		else
		{
			throw "Invalid robot linear velocity: AutonomousFlipperControl.cpp";
		}

	}

	if (laserAvailable and imuAvailable and !rotate)
	{
		switch (flipperMode)
		{
			// Snake mode
			case SNAKE:
			{
				snakeModeObject.DistVect(front,rear);
				snakeModeObject.SetUss(front_uss,mid1_uss,mid2_uss,rear_uss);
				snakeModeObject.snakeMode(frontDesiredAngle,rearDesiredAngle);
				pubAng = true;
				break;
			}
				// Step mode
			case UPWARD:
			{
				stepBehavior.DistVect(front,rear,currentFlipperAngleFront,currentFlipperAngleRear,robotDirection);
				stepBehavior.SetUss(front_uss,mid1_uss,mid2_uss,rear_uss);
				stepBehavior.upwardStepMode (frontDesiredAngle,rearDesiredAngle);
				pubAng = true;
				break;
			}
				// Step mode (downward)
			case DOWNWARD:
			{
				stepBehavior.DistVect(front,rear,currentFlipperAngleFront,currentFlipperAngleRear,robotDirection);
				stepBehavior.SetUss(front_uss,mid1_uss,mid2_uss,rear_uss);
				stepBehavior.downwardStepMode (frontDesiredAngle,rearDesiredAngle);
				pubAng = true;
				break;
			}
			case STAIRSUP:
			{
				// static values of uni stairs
				stairBehavior.DistVect(front,rear,front_rotBodyAngle,rear_rotBodyAngle);
				stairBehavior.SetUss(front_uss,mid1_uss,mid2_uss,rear_uss);
				stairBehavior.stairUpMode (33 * D2R, 0.2,frontDesiredAngle,rearDesiredAngle);
				pubAng = true;
				break;
			}
			case STAIRSDOWN:
			{
				// static values of uni stairs
				stairBehavior.DistVect(front,rear,front_rotBodyAngle,rear_rotBodyAngle);
				stairBehavior.SetUss(front_uss,mid1_uss,mid2_uss,rear_uss);
				stairBehavior.stairDownMode (27.0 * D2R, 0.16,frontDesiredAngle,rearDesiredAngle);
				pubAng = true;
				break;

			}
		}
	}

	if (selectDriveBackward == true)
	{
		publishVelocities ();
	}
	if (pubAng)
		publishAngles ();
	flipperModeOld = flipperMode;
}




void RobotControl::publishVelocities ()
{
	stairBehavior.MoveRobot(moveRobot);
	geometry_msgs::Twist velocities;
	if (moveRobot == true)
	{

		if (selectDriveForward == true)
		{
			velocities.angular.z = 0;
			velocities.linear.x = 0.05;
		}
		else
		{
			velocities.angular.z = 0;
			velocities.linear.x = -0.1;
		}
	}
	else
	{
		velocities.angular.z = 0.0;
		velocities.linear.x = 0.0;
	}
	//velPub.publish (velocities);
}
void RobotControl::publishAngles ()
{
	if (frontDesiredAngle != NAN)
	{
		frontFlipperAngleDesiredMsg.data = -frontDesiredAngle * D2R - 0.2;
	}
	else
		ROS_INFO (" Front angle NAN:\t  [%7.3f]", frontDesiredAngle);
	if (rearDesiredAngle != NAN)
	{
		rearFlipperAngleDesiredMsg.data = -rearDesiredAngle * D2R - 0.2;
	}
	else
		ROS_INFO (" Rear angle NAN:\t  [%7.3f]", rearDesiredAngle);

	if(frontFlipperAngleDesiredMsg.data<=M_PI/2 && frontFlipperAngleDesiredMsg.data>=-M_PI/2)
	{
		frontFlipperAngleDesiredPub.publish (frontFlipperAngleDesiredMsg);
	}

	if(rearFlipperAngleDesiredMsg.data<=M_PI/2 && rearFlipperAngleDesiredMsg.data>=-M_PI/2)
	{
		rearFlipperAngleDesiredPub.publish (rearFlipperAngleDesiredMsg);

	}
}

void RobotControl::print_Msg ()
{
	ROS_INFO ("\t F R O N T:\t\t\t\t R E A R:");

	for (unsigned int row = 0; row < rear.x.size (); row++)
	{
		if (row < front.x.size () - 1)
		{
			ROS_INFO ("\t X = [%7.3f] \t Z = [%7.3f] \t\t X = [%7.3f] \t Z = [%7.3f]", front.x.at (row), front.z.at (row), rear.x.at (row), rear.z.at (row));
		}
		else
		{
			ROS_INFO ("\t\t\t\t \t\t X = [%7.3f] \t Z = [%7.3f]", rear.x.at (row), rear.z.at (row));
		}
	}
	std::cout << std::endl << std::endl;
}

void RobotControl::reconfigureCallback (robot_navigation::StepConfig &confg, uint32_t level)
{
	ROS_INFO ("************************************ Reconfigure callback ********************************************\n");

	startStairBehavior= confg.startStairBehavior;
	snake_mode_front_angle_min = confg.snake_mode_front_angle_min;
	snake_mode_rear_angle_min = confg.snake_mode_rear_angle_min;

	body_front_0_distance_offset = confg.body_front_0_distance_offset;
	body_front_1_distance_offset = confg.body_front_1_distance_offset;
	body_rear_0_distance_offset = confg.body_rear_0_distance_offset;
	body_rear_1_distance_offset = confg.body_rear_1_distance_offset;

	flipper_angle_min = confg.flipper_angle_min;
	flipper_angle_max = confg.flipper_angle_max;
	laser_scan_front_angle_min = D2R * confg.laser_scan_front_angle_min;
	laser_scan_front_angle_max = D2R * confg.laser_scan_front_angle_max;
	laser_scan_rear_angle_min = D2R * confg.laser_scan_rear_angle_min;
	laser_scan_rear_angle_max = D2R * confg.laser_scan_rear_angle_max;
	Hoku_scan_angle_min = D2R * confg.Hoku_scan_angle_min;
	Hoku_scan_angle_max = D2R * confg.Hoku_scan_angle_max;

	stairBehavior.SetStairBehaviorParams(flipper_angle_max,flipper_angle_min);
	snakeModeObject.SetSnakeModeParams(flipper_angle_max,flipper_angle_min);
	stepBehavior.SetStepBehaviorParams(flipper_angle_max,flipper_angle_min);
}

bool RobotControl::toggleModeCallback (std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	selectMode = !selectMode;
	ROS_INFO ("Changed MODE!!!!!!");

	if (flipperMode < 6)
		flipperMode++;
	else
		flipperMode = 0;

	if (flipperMode != 1)
		velocityLimitation = false;

	return true;
}

bool RobotControl::selectModeCallback (get_std_msgs::IntegerBool::Request &req,
		get_std_msgs::IntegerBool::Response &res)
{
	if(req.request<6 && req.request>=0)
	{
		flipperMode=req.request;
		if (flipperMode != 1)
		{
				velocityLimitation = false;
		}
		return true;
	}
	return false;
}
bool RobotControl::selectDriveForwardCallback (std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	selectDriveForward = !selectDriveForward;
	ROS_INFO ("Changed selectDriveForward!!!!!!");

	return true;
}
bool RobotControl::objectSelectedCallback(std_srvs::Empty::Request &req,
		std_srvs::Empty::Response &res)
{
	startStairBehavior=!startStairBehavior;
	return true;
}

bool RobotControl::selectDriveBackwardCallback (std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	selectDriveBackward = !selectDriveBackward;
	ROS_INFO ("Changed selectDriveBackward!!!!!!");

	return true;
}
