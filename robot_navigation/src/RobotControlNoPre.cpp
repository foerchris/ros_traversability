#include "RobotControl.h"

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
	S = L + R;
	s = (L / 2.0) + R;

	base_offset = 0.0;

	linearVel = 0.0;
	angularVel = 0.0;
	stepBeta = 0.0;
	stepZ = 0.0;
	imuAvailable = false;
	laserAvailable = false;
	velocityLimitationOld = false;
	velocityLimitation = false;
	stepInit = false;
	stepClimb = false;
	stepClimbComplete = false;
	stepBorderReached = false;
	flipperMode = 0;
	flipperModeOld = flipperMode;
	stepDetected = -1;
	robotDirection = 0;
	stateStep = -1;
	previousStateStep = stateStep;
	angleThreshold = -8.0;
	bodyAngleThreshold = 6.0;
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

	currentState=searchStep;

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
	roll = roll * R2D;
	pitch = pitch * R2D;
	yaw = yaw * R2D;
	bodyAngle = (round (pitch * 10.0)) / 10.0;
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
				if ((not std::isnormal (range)) || (range <= flyingPointsFiltered.range_min) || (range >= flyingPointsFiltered.range_max))
					range = flyingPointsFiltered.range_max;
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
					ROS_ERROR ("laserFilter (front) %s", ex.what ());
					return;
				}
				front_rotBodyAngle.x.push_back (pointTransformed2.getX () - s);
				front_rotBodyAngle.z.push_back (pointTransformed2.getZ () + base_offset);

				front.x.push_back (pointTransformed.getX () - s);
				front.z.push_back (pointTransformed.getZ () + base_offset);
			}
			// Transformation for rear values
			if ((angle <= laser_scan_rear_angle_min) and (angle >= laser_scan_rear_angle_max) and !(std::isnan (range)))
			{
				if ((not std::isnormal (range)) || (range <= flyingPointsFiltered.range_min) || (range >= flyingPointsFiltered.range_max))
				{
					range = flyingPointsFiltered.range_max; //
				}

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
			angle += flyingPointsFiltered.angle_increment;
		}
		laserAvailable = true;



		currentMode ();
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

			if ((not std::isnormal (range)) || (range <= Hoku_angle.range_min) || (range >= Hoku_angle.range_max))
				range = Hoku_angle.range_max;

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
			front_horizontal.z.push_back (pointTransformed.getZ () + base_offset);
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
	//std::cout<<"elementCount"<<elementCount<<std::endl;
	//std::cout<<"range"<<range<<std::endl;

	for (std::size_t i = 0; i < range; i++)
	{
		std::size_t j = 0;
		//std::cout<<"i"<<i<<std::endl;
		for (int index = i - medianFilterRadius; index <= i + medianFilterRadius; index++)
		{
			int idx = index;
		//	std::cout<<"index"<<index<<std::endl;
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
		//	std::cout<<"idx"<<idx<<std::endl;

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
		stepReset ();
	}

	int downwardStep = 99;
	if (laserAvailable)
		downwardStep = downwardStepDetection (rear);

	ROS_INFO_STREAM ("\n" << "*** flipperMode:\t" << flipperMode << "\n" << "*** stepDetected:\t" << stepDetected << "\n" << "*** stepInit:\t" << stepInit << "\n" << "*** stepClimb:\t" << stepClimb << "\n" << "*** stepBeta:\t" << stepBeta << "\n" << "*** stepZ:\t" << stepZ << "\n" << "*** robotDirection\t" << robotDirection << "\n" << "*** rear: downwardStep\t" << downwardStep << "\n" << "*** velocityLimitation:\t" << velocityLimitation << "\n" << "*** velocityLimitationOld:\t" << velocityLimitationOld);

	///double calcPhi = (M_PI_2 - acos ((L * sin (bodyAngle * D2R) + R * (1 - sin (bodyAngle * D2R))) / (l + r))) * R2D;

	ROS_INFO (" Ultrasound front: [%7.3f] \t mid1: [%7.3f] ", front.vd, mid1_uss);
	ROS_INFO (" Ultrasound  rear: [%7.3f] \t mid2: [%7.3f] ", rear.vd, mid2_uss);
	switch (flipperMode)
	{
		case SNAKE:
			ROS_INFO (" SNAKE: [ON] \t mode: [Snake] \t\t body Angle: [%7.2f]", bodyAngle);
			break;
			// Snake mode
		case UPWARD:
			ROS_INFO (" UPWARD: [ON] \t mode: [Step up] \t\t body Angle: [%7.2f]", bodyAngle);
			break;
		case DOWNWARD:
			ROS_INFO (" DOWNWARD: [ON] \t mode: [Step down] \t\t body Angle: [%7.2f]", bodyAngle);
			break;
			// No mode selected
		case STAIRSUP:
			ROS_INFO (" STAIRSUP: [ON] \t mode: [Step up] \t\t body Angle: [%7.2f]", bodyAngle);
			break;
		case STAIRSDOWN:
			ROS_INFO (" STAIRSDOWN: [ON] \t mode: [Step down] \t\t body Angle: [%7.2f]", bodyAngle);
			break;
			// No mode selected

		default:
			ROS_INFO (" Autonomous flipper control: [OFF]");
	}

	if (angularVel != 0.0 and linearVel == 0.0)
	{
		double phi = 90.0;
		ROS_INFO ("Robot is rotating --> Setting all flippers upright: desired angle: [%7.2f].", phi);
		frontDesiredAngle = phi;
		rearDesiredAngle = phi;
		robotDirection = 0;
		stepReset ();
		pubAng = true;
		rotate = true;

	}
	else
	{

		// Robot moves forward
		if ((linearVel > 0.0) or ((linearVel == 0.0) and (robotDirection > 0)))
		{
			if (robotDirection < 0)
				stepReset ();

			robotDirection = 1;
		}
		// Robot moves backwards
		else if ((linearVel < 0.0) or ((linearVel == 0.0) and (robotDirection < 0)))
		{
			if (robotDirection > 0)
				stepReset ();

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

				snakeMode ();
				pubAng = true;
				break;
			}
				// Step mode
			case UPWARD:
			{
				upwardStepMode ();
				pubAng = true;
				break;
			}
				// Step mode (downward)
			case DOWNWARD:
			{
				downwardStepMode ();
				pubAng = true;
				break;
			}
			case STAIRSUP:
			{
				// static values of uni stairs
				stairUpMode (27.0 * D2R, 0.16);
				pubAng = true;
				break;
			}
			case STAIRSDOWN:
			{
				// static values of uni stairs
				stairDownMode (27.0 * D2R, -0.16);
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

/**
 * Mode for driving a stair down
 * @param stairAngle slope angle of the stair
 * @param stepHeight height of the first step edge
 */
void RobotControl::stairDownMode (double stairAngle, double stepHeight)
{

	std::cout << "---------stairDownMode-------- " << std::endl;

	//double deltaSubtrackAngle = -7.0;

	base_offset = 0.0;
	double deltaSubtrackAngle = 0.0;
	double distanceUssMax = 0.20;
	double distanceUssMin = 0.10;

	///double distanceUltrasoundSensor = 0.33;

	double distanceUssMidThreshold = 0.10; // distance to ground
	///double distanceRearThreshold = 0.30; // distance to rear
	///double bodyAngThreshold = 3.0;
	///static bool upWards = false;

	VecXZPhi desiredVecFront, desiredVecRear;

	enum stairUpEnum
	{
		start, ptichUp, startClimbing, normalClimbing, finalClimbing, pitchDown, reset
	};
	////static int stairUpStat = -1;
	static int state = start;
	double epsilonAngle = 3.0 * D2R;
	int newState = -1;
	///double friction = 0.8;
	double deltaAngleAbs = std::fabs (bodyAnglePitch - stairAngle);
	double deltaAngle = stairAngle - bodyAnglePitch;
	double marginAngle = 5.0;
	///double desiredAngularVel;
	///double deltadirectionAngle;
	///double controlGain;
	///double stairAngleNDeg = -stairAngle * R2D;
	double flipper_angle_max_temp = 45;
	double flipper_angle_min_temp = -90;

	///double bodyAnglePitchZero = bodyAnglePitch;
	///double d_0; // offset Distance [m]
	///double TestdesiredVecFront;
	///double static TestdesiredVecRear;
	double Kp = 0.1; //0.03
	double Ki = 0.02; //10
	double Kd = 0.01; // 2
	double dt;
	double errorTerm;
	double pidAngle;
	static double phi = 0;
	int endOfStair;
	DistanceVec RearStair;
	std::vector<double> RearX;
	std::vector<double> RearZ;
	currentTime = ros::Time::now ().toSec ();
	std::cout << "deltaAngle: " << deltaAngle * R2D << std::endl;
	std::cout << "epsilonAngle: " << epsilonAngle * R2D << std::endl;
	std::cout << "bodyAnglePitch: " << bodyAnglePitch * R2D << std::endl;
	std::cout << "previousTime: " << previousTime << std::endl;
	std::cout << "currentTime: " << currentTime << std::endl;
	if (front_uss <= distanceUssMin && rear_uss <= distanceUssMin && fabs (bodyAngle) <= epsilonAngle * R2D)
	{
		state = start;
	}
	switch (state)
	{
		case start:
			snakeModeVec (rear, desiredVecRear);
			;
			desiredVecFront.phi = deltaSubtrackAngle;
			if (front_uss >= distanceUssMin)
			{
				moveRobot = false;
				newState = ptichUp;
				phi = 0;
			}
			break;

		case ptichUp:

			dt = (currentTime - previousTime);
			errorTerm = -deltaAngle * R2D;
			pidAngle = Kp * errorTerm + Ki * dt * errorTerm + Kd * errorTerm / dt;
			std::cout << "errorTerm: " << errorTerm << " pidAngle: " << pidAngle << std::endl;

			previousTime = currentTime;
			if (pidAngle <= 10 && pidAngle >= -10)
			{
				phi = phi + 2 * pidAngle;
				std::cout << "phi: " << phi << std::endl;

				phi = (phi < flipper_angle_max_temp) ? phi : flipper_angle_max_temp;
				phi = (phi > flipper_angle_min_temp) ? phi : flipper_angle_min_temp;
			}

			std::cout << "phi: " << phi << std::endl;
			moveRobot = false;
			desiredVecFront.phi = deltaSubtrackAngle - marginAngle;

			desiredVecRear.phi = phi;
			if (deltaAngle <= (2 * epsilonAngle) || (rearDesiredAngle >= 80.0 || rear_uss >= distanceUssMax))
			{
				newState = startClimbing;

			}
			break;

		case startClimbing:
			desiredVecFront.phi = deltaSubtrackAngle;
			dt = (currentTime - previousTime);
			errorTerm = -deltaAngle * R2D;
			pidAngle = Kp * errorTerm + Ki * dt * errorTerm + Kd * errorTerm / dt;
			std::cout << "errorTerm: " << errorTerm << " pidAngle: " << pidAngle << std::endl;
			pidAngle = (pidAngle < 10.0) ? pidAngle : 10.0;
			pidAngle = (pidAngle > -10.0) ? pidAngle : -10.0;
			phi = phi + 2 * pidAngle;
			std::cout << "phi: " << phi << std::endl;
			phi = (phi < flipper_angle_max_temp) ? phi : flipper_angle_max_temp;
			phi = (phi > flipper_angle_min_temp) ? phi : flipper_angle_min_temp;
			std::cout << "phi: " << phi << std::endl;
			desiredVecFront.phi = deltaSubtrackAngle - marginAngle;
			desiredVecRear.phi = phi;
			previousTime = currentTime;
			std::cout << "deltaAngleAbs: " << deltaAngleAbs << std::endl;
			std::cout << "epsilonAngle: " << epsilonAngle << std::endl;
			endOfStair = upwardEndOfStairDetection (front_rotBodyAngle);
			std::cout << "endOfStair: " << endOfStair << std::endl;
			if (deltaAngleAbs <= epsilonAngle && rear_uss <= distanceUssMin)
			{
				newState = normalClimbing;
			}
			break;

		case normalClimbing:

			desiredVecFront.phi = deltaSubtrackAngle;
			desiredVecRear.phi = deltaSubtrackAngle;

			endOfStair = upwardEndOfStairDetection (front_rotBodyAngle);
			std::cout << "upwardEndOfStairDetection (front_rotBodyAngle): " << endOfStair << std::endl;
			if (endOfStair == true && front_uss <= distanceUssMin)
			{
				newState = finalClimbing;
			}
			break;
		case finalClimbing:
			desiredVecFront.phi = deltaSubtrackAngle + 2 * marginAngle;
			desiredVecRear.phi = deltaSubtrackAngle - marginAngle;

			if (deltaAngle >= (2 * epsilonAngle) && front_uss >= distanceUssMidThreshold)
			{
				newState = pitchDown;
				moveRobot = false;
				previousTime = currentTime;
			}
			break;

		case pitchDown:
			desiredVecFront.phi = deltaSubtrackAngle;
			snakeModeVec (rear, desiredVecRear);
			if ((currentTime - previousTime) <= 1.0)
				moveRobot = false;
			if (rear_uss <= distanceUssMin && (currentTime - previousTime) >= 1.0)
			{
				newState = reset;
			}
			break;

		case reset:
			desiredVecFront.phi = bodyAngle + deltaSubtrackAngle + marginAngle;
			snakeModeVec (rear, desiredVecRear);

			if (fabs (bodyAnglePitch) <= epsilonAngle)
			{
				newState = start;
			}
			break;
		default:
			break;
	}
	if (newState != -1)
		state = newState;
	frontDesiredAngle = desiredVecFront.phi;
	rearDesiredAngle = desiredVecRear.phi;
	ROS_INFO (" stairAngle %f", stairAngle * R2D);
	ROS_INFO (" bodyAngleRoll %f", bodyAngleRoll * R2D);
	ROS_INFO (" state %i", state);
	ROS_INFO (" newState %i", newState);
	ROS_INFO (" Front flipper:\t desired angle: [%7.3f]", desiredVecFront.phi);
	ROS_INFO ("  Rear flipper:\t desired angle: [%7.3f]", desiredVecRear.phi);

}
/**
 * Mode to drive a stair up
 * @param stairAngle slope angle of the stair
 * @param stepHeight height of the first step edge
 */
void RobotControl::stairUpMode (double stairAngle, double stepHeight)
{


	//double deltaSubtrackAngle = -7.0;
	base_offset = 0.0;
	double deltaSubtrackAngle = 0.0;
	double distanceUssMax = 0.20;
	double distanceUssMin = 0.10;
	moveRobot = true;

	double distanceUltrasoundSensor = 0.33;

	///double distanceUssMidThreshold = 0.10; // Schwellwert  zum Boden
	///double distanceRearThreshold = 0.30; // Schwellwert nach  Hinten
	///double bodyAngThreshold = 3.0;
	double bodyAngleUtrasoundSensor = -atan2 (mid1_uss - mid2_uss, distanceUltrasoundSensor);
	///static bool upWards = false;

	VecXZPhi desiredVecFront, desiredVecRear;

	enum stairUpEnum
	{
		start, ptichUp, normalClimbing, finalClimbing, pitchDown, reset
	};
	///static int stairUpStat = -1;
	static int state = start;
	double epsilonAngle = 3.0 * D2R;
	int newState = -1;
	///double friction = 0.8;
	double marginAngle = 5.0;
	double deltaAngle = std::fabs (bodyAnglePitch - stairAngle);
	double deltaAngle2 = bodyAnglePitch - stairAngle;
	double deltaAngleOffset = bodyAnglePitch - stairAngle - marginAngle;

	///double desiredAngularVel;
	///double deltadirectionAngle;
	///double controlGain;
	///double stairAngleNDeg = -stairAngle * R2D;
	double Kp = 0.01; //0.03
	double Ki = 0.02; //10
	double Kd = 0.01; // 2
	double dt;
	double errorTerm;
	double static lastErrorTerm = 0.0;
	double static integral = 0.0;
	double pidAngle;
	static double phi = 0;
	///double flipper_angle_max_temp = 45;
	///double flipper_angle_min_temp = -80;
	///double bodyAnglePitchZero = bodyAnglePitch;
	///double d_0; // offset Distance [m]
	///double TestdesiredVecFront;
	///double static TestdesiredVecRear;
	int endOfStair;
	DistanceVec RearStair;
	std::vector<double> RearX;
	std::vector<double> RearZ;
	currentTime = ros::Time::now ().toSec ();
	std::cout << "deltaAngle2: " << deltaAngle2 * R2D << std::endl;
	std::cout << "epsilonAngle: " << epsilonAngle * R2D << std::endl;
	std::cout << "bodyAnglePitch: " << bodyAnglePitch * R2D << std::endl;
	std::cout << "previousTime: " << previousTime << std::endl;
	std::cout << "currentTime: " << currentTime << std::endl;
	if (front_uss <= distanceUssMin && rear_uss <= distanceUssMin && fabs (bodyAngle) <= epsilonAngle * R2D)
	{
		state = start;
	}
	switch (state)
	{
		case start:
			desiredVecRear.phi = snakeModeAngle (stepHeight);
			desiredVecFront.phi = bodyAngle + deltaSubtrackAngle + marginAngle;
			if (deltaAngle >= epsilonAngle && fabs (bodyAnglePitch) >= epsilonAngle)
			{
				newState = ptichUp;
			}
			break;

		case ptichUp:
			desiredVecFront.phi = bodyAngle + deltaSubtrackAngle + marginAngle;
			desiredVecRear.phi = snakeModeAngle (stepHeight);
			if (deltaAngle <= epsilonAngle)
			{
				newState = normalClimbing;
			}
			break;

		case normalClimbing:
			desiredVecFront.phi = deltaSubtrackAngle;
			desiredVecRear.phi = deltaSubtrackAngle;
			endOfStair = upwardEndOfStairDetection (rear_rotBodyAngle);
			std::cout << "upwardEndOfStairDetection (rear_rotBodyAngle): " << endOfStair << std::endl;
			if ((endOfStair == true && (rear_uss <= distanceUssMin || mid2_uss <= distanceUssMin)))
			{
				newState = finalClimbing;
				moveRobot = false;
				lastErrorTerm = 0.0;
				integral = 0.0;
				phi = deltaSubtrackAngle;
			}
			if (endOfStair == true && deltaAngle >= epsilonAngle)
			{
				newState = reset;
				moveRobot = false;
				lastErrorTerm = 0.0;
				integral = 0.0;
				phi = deltaSubtrackAngle;
			}

			break;

		case finalClimbing:

			std::cout << "phi: " << phi << std::endl;
			dt = (currentTime - previousTime);
			errorTerm = deltaAngleOffset * R2D;
			integral += errorTerm * dt;
			pidAngle = Kp * errorTerm + Ki * integral + Kd * (errorTerm - lastErrorTerm) / dt;
			std::cout << "errorTerm: " << errorTerm << " pidAngle: " << pidAngle << std::endl;
			lastErrorTerm = errorTerm;
			previousTime = currentTime;
			pidAngle = (pidAngle < 0.0) ? pidAngle : 0.0;
			pidAngle = (pidAngle > -60.0) ? pidAngle : -60.0;
			phi = pidAngle;
			std::cout << "phi: " << phi << std::endl;

			desiredVecRear.phi = phi;
			desiredVecFront.phi = deltaSubtrackAngle;

			std::cout << "deltaAngle2: " << deltaAngle2 * R2D << " epsilonAngle: " << epsilonAngle * R2D << std::endl;

			if (deltaAngle2 >= epsilonAngle)
			{
				newState = pitchDown;
				previousTime = currentTime;
				lastErrorTerm = 0.0;
				integral = 0.0;
			}
			break;

		case pitchDown:
			desiredVecRear.phi = phi;
			desiredVecFront.phi = deltaSubtrackAngle;

			std::cout << "0.8 * stairAngle: " << 0.8 * stairAngle << std::endl;
			std::cout << "bodyAnglePitch: " << bodyAnglePitch << std::endl;
			if (front_uss <= distanceUssMin && bodyAnglePitch <= (0.8 * stairAngle))
			{
				newState = reset;
				previousTime = currentTime;
			}
			break;

		case reset:

			std::cout << "phi: " << phi << std::endl;
			dt = (currentTime - previousTime);
			errorTerm = bodyAnglePitch - epsilonAngle;
			integral += errorTerm * dt;
			pidAngle = Kp * errorTerm + Ki * integral + Kd * (errorTerm - lastErrorTerm) / dt;

			std::cout << "errorTerm: " << errorTerm << " pidAngle: " << pidAngle << std::endl;
			lastErrorTerm = errorTerm;
			previousTime = currentTime;
			pidAngle = (pidAngle < 10.0) ? pidAngle : 10.0;
			pidAngle = (pidAngle > -10.0) ? pidAngle : -10.0;
			phi = phi + pidAngle * 1000;
			std::cout << "phi: " << phi << std::endl;
			std::cout << "phi: " << phi << std::endl;

			desiredVecFront.phi = bodyAngle + deltaSubtrackAngle;
			desiredVecRear.phi = phi;
			if (fabs (bodyAnglePitch) <= epsilonAngle)
			{
				newState = start;
			}
			if (front_uss <= distanceUssMin && bodyAnglePitch <= (0.7 * stairAngle))
				newState = start;
			break;
		default:
			break;
	}
	if (newState != -1)
		state = newState;
	frontDesiredAngle = desiredVecFront.phi;
	rearDesiredAngle = desiredVecRear.phi;

	//ROS_INFO (" distancePitch %f", distancePitch);
	ROS_INFO (" stairAngle %f", stairAngle * R2D);
	ROS_INFO (" bodyAngleRoll %f", bodyAngleRoll * R2D);
	ROS_INFO (" bodyAngle: [%3.3f] \t\t bodyAngleUtrasoundSensor: [%3.3f]", bodyAngle, bodyAngleUtrasoundSensor * R2D);
	ROS_INFO (" state %i", state);
	ROS_INFO (" newState %i", newState);
	ROS_INFO (" Front flipper:\t desired angle: [%7.3f]", desiredVecFront.phi);
	ROS_INFO ("  Rear flipper:\t desired angle: [%7.3f]", desiredVecRear.phi);

}

/**
 * Runs all function for snake mode
 */
void RobotControl::snakeMode ()
{
	VecXZPhi desiredVecFront;
	VecXZPhi desiredVecRear;


	snakeModeVec (front, desiredVecFront);

	snakeModeVec (rear, desiredVecRear);

	desiredVecFront.phi = (desiredVecFront.phi > -25.0) ? desiredVecFront.phi : -25.0;


	if (mid2_uss >= 0.05 && rear.vd >= 0.08)
	{
		desiredVecRear.phi = (desiredVecRear.phi > -60.0) ? desiredVecRear.phi : -60.0;
	}
	else
	{
		desiredVecRear.phi = (desiredVecRear.phi > -25.0) ? desiredVecRear.phi : -25.0;
	}

	ROS_INFO (" Front flipper:\t desired angle: [%7.3f] @ height: [%7.3f] and distance: [%7.3f]", desiredVecFront.phi, desiredVecFront.z, desiredVecFront.x);
	ROS_INFO (" Rear flipper:\t\t desired angle:  [%7.3f] @ height: [%7.3f] and distance: [%7.3f]", desiredVecRear.phi, desiredVecRear.z, desiredVecRear.x);
	frontDesiredAngle = desiredVecFront.phi;
	rearDesiredAngle = desiredVecRear.phi;

	double distanceUltrasoundSensor = 0.33;
	double bodyAngleUtrasoundSensor = -atan2 (mid1_uss - mid2_uss, distanceUltrasoundSensor);
	ROS_INFO (" bodyAngle: [%3.3f] \t\t bodyAngleUtrasoundSensor: [%3.3f]", bodyAngle, bodyAngleUtrasoundSensor * R2D);
}

/**
 * Gets the height and distance of the next obstacle
 * @param distanceVec values of the laser scanner
 * @param desiredVec values of the next obstacle and desired flipper angle
 */
void RobotControl::snakeModeVec (DistanceVec distanceVec, VecXZPhi &desiredVec)
{
	if (!distanceVec.z.size())
	{
		return;
	}

	std::vector<double>::iterator maxZIt;
	unsigned int maxZEl;
	double maxZ;
	double xAtMaxZ;
	double phi;

	maxZIt = std::max_element (distanceVec.z.begin (), distanceVec.z.end ());
	maxZEl = std::distance (distanceVec.z.begin (), maxZIt);
	maxZ = distanceVec.z.at (maxZEl);

	xAtMaxZ = distanceVec.x.at (maxZEl);

	phi = snakeModeAngle (maxZ);
	desiredVec.x = xAtMaxZ;
	desiredVec.z = maxZ;
	desiredVec.phi = phi;

}
/**
 * Calculates the flipper angle for an obstacle.
 * @param z height of the obstacle
 * @return angle for the flipper
 */
double RobotControl::snakeModeAngle (double z)
{
	double phi;
	double arg = (z - R) / d;

	if (std::fabs (arg) > 1.0)
		arg = (arg < 0) ? -1.0 : 1.0;

	phi = acos (arg) - theta;
	phi = 90.0 - (R2D * phi);
	ROS_INFO_STREAM (" phi: " << phi << "@ height: " << z);
	phi = (phi < flipper_angle_max) ? phi : flipper_angle_max;
	phi = (phi > flipper_angle_min) ? phi : flipper_angle_min;
	return phi;
}

/**
 * Detects a step in down direction
 * @param vec values of the laser scanner
 * @return true if it is a down step
 */
int RobotControl::downwardStepDetection (DistanceVec vec)
{

	int sizeVec = vec.z.size ();
	for (int row = 0; row < sizeVec - 1; row++)
	{
		int dz = std::fabs (vec.z.at (row) - vec.z.at (row + 1));
		//std::cout<<"step_height_treshold"<<step_height_treshold<<std::endl;
		if (dz >= step_height_treshold)
		{
			std::vector<double>::iterator minIt;
			double minVel;
			unsigned int minEl;
			minIt = std::min_element (vec.z.begin (), vec.z.end ());
			minEl = std::distance (vec.z.begin (), minIt);
			minVel = vec.z.at (minEl);
			//std::cout<<"minVel"<<minVel<<std::endl;
			if (minVel <= -step_height_treshold)
				return 1;
		}
	}
	return 0;
}

/**
 * Mode to chose driving forward or backward a step down
 */
void RobotControl::downwardStepMode ()
{
	VecXZPhi desiredVecFront;
	VecXZPhi desiredVecRear;

	switch (robotDirection)
	{
		// Robot moves backwards
		case -1:
		{
			rear.vd=mid2_uss;
			downwardStepModeVec (rear, front, desiredVecRear, desiredVecFront);
			break;
		}
			// Robot moves forward
		case 1:
		{
			front.vd=mid1_uss;
			downwardStepModeVec (front, rear, desiredVecFront, desiredVecRear);
			break;
		}
			// Snake mode
		default:
		{
			snakeModeVec (front, desiredVecFront);
			snakeModeVec (rear, desiredVecRear);
		}
	}

	ROS_INFO (" Front flipper:\t desired angle: [%7.3f] @ height: [%7.3f] and distance: [%7.3f]", desiredVecFront.phi, desiredVecFront.z, desiredVecFront.x);
	ROS_INFO ("  Rear flipper:\t desired angle: [%7.3f] @ height: [%7.3f] and distance: [%7.3f]", desiredVecRear.phi, desiredVecRear.z, desiredVecRear.x);
	frontDesiredAngle = desiredVecFront.phi;
	rearDesiredAngle = desiredVecRear.phi;
}

/**
 * Drives a step down.
 * @param distanceVecFront front vales of the laser scanner
 * @param distanceVecRear rear values of the laser scanner
 * @param desiredVecFront values for the front flipper
 * @param desiredVecRear vaules for the rear flipper
 */
void RobotControl::downwardStepModeVec (DistanceVec distanceVecFront, DistanceVec distanceVecRear, VecXZPhi &desiredVecFront, VecXZPhi &desiredVecRear)
{
	switch (stateStep)
	{
		case START:
		{
			std::cout<<"START"<<std::endl;
			if(downwardStepDetection (distanceVecFront))
			{
				stateStep = SEARCH;
			}
			break;
		}
		case SEARCH:
		{
			std::cout<<"SEARCH"<<std::endl;
			std::vector<double>::iterator maxZIt;
			unsigned int maxZEl;

			maxZIt = std::max_element (distanceVecFront.z.begin (), distanceVecFront.z.end ());
			maxZEl = std::distance (distanceVecFront.z.begin (), maxZIt);
			stepZ = distanceVecFront.z.at (maxZEl);
			if (stepZ <= -step_height_treshold)
			{
				stateStep = DETECTED;
			}
			break;
		}
		case DETECTED:
		{
			std::cout<<"DETECTED"<<std::endl;
			snakeModeVec (distanceVecRear, desiredVecRear);
			if((distanceVecFront.vd <= step_height_treshold))
			{
				stateStep = MOVE_FORWARD_FLIPPER;
			}
			break;
		}
		case MOVE_FORWARD_FLIPPER:
		{
			std::cout<<"MOVE_FORWARD_FLIPPER"<<std::endl;
			snakeModeVec (distanceVecFront, desiredVecFront);
			if(distanceVecRear.vd <= step_height_treshold)
			{
				stateStep = MOVE_BACKWARD_FLIPPER;
			}
			break;
		}
		case MOVE_BACKWARD_FLIPPER:
		{
			std::cout<<"MOVE_BACKWARD_FLIPPER"<<std::endl;
			double tempTreshholdDistanceRearX = -0.1;
			double tempTreshholdDistanceRearUltrasound = 0.13;
			distanceVecFront.z = front.z;
			distanceVecFront.x = front.x;
			distanceVecRear.z = rear.z;
			distanceVecRear.x = rear.x;
			snakeModeVec (distanceVecFront, desiredVecFront);
			snakeModeVec (distanceVecFront, desiredVecFront);

			if ((mid2_uss > tempTreshholdDistanceRearUltrasound) and (distanceVecRear.vd < tempTreshholdDistanceRearUltrasound) and desiredVecRear.x >= tempTreshholdDistanceRearX)
			{
				stateStep = FINISHED;
			}
			break;
		}
		case FINISHED:
		{
			std::cout<<"--- FINISHED "<<std::endl;
			stepReset ();
			break;
		}
		default:
		{
			snakeModeVec (distanceVecFront, desiredVecFront);
			snakeModeVec (distanceVecRear, desiredVecRear);
			double Mid1_ussDistance = 0.11;
			double Mid2_ussDistance = 0.11;

			if (flipperMode != 0 && frontDesiredAngle <= angleThreshold && mid1_uss <= Mid1_ussDistance)
				frontDesiredAngle = NAN;
			if (flipperMode != 0 && rearDesiredAngle <= angleThreshold && mid2_uss <= Mid2_ussDistance)
				rearDesiredAngle = NAN;
			break;
		}
	}
}

/**
 * Mode to drive a step up in front or rear direction
 */
void RobotControl::upwardStepMode ()
{
	VecXZPhi desiredVecFront;
	VecXZPhi desiredVecRear;
	desiredVecFront.phi=NAN;
	desiredVecRear.phi=NAN;
	switch (robotDirection)
	{
		// Robot moves backwards
		case -1:
		{
			front.vd=mid1_uss;
			upwardStepModeVec (rear, front, desiredVecRear, desiredVecFront,currentFlipperAngleFront,currentFlipperAngleRear);
			break;
		}
			// Robot moves forward
		case 1:
		{
			rear.vd=mid2_uss;
			upwardStepModeVec (front, rear, desiredVecFront, desiredVecRear,currentFlipperAngleRear,currentFlipperAngleFront);
			break;
		}
			// Snake mode
		default:
		{
			snakeModeVec (front, desiredVecFront);
			snakeModeVec (rear, desiredVecRear);
			std::cout<<"// Snake mode "<< desiredVecRear.phi	<<std::endl;

		}

	}
	frontDesiredAngle = desiredVecFront.phi;
	rearDesiredAngle = desiredVecRear.phi;
}

/**
 * Detects the end of a stair in upward direction
 * @param vec values of the laser scanner
 * @return true if end is detected
 */
bool RobotControl::upwardEndOfStairDetection (DistanceVec vec)
{

	int N = vec.z.size () - 1;

	double stepHeight = vec.vd;
	double epsilonHeight = 0.5 * stepHeight;
	double maxHeightDif = 0.03;
	///double epsilonDelatHeight;
	double maxDistance = 0.8;
	if (N >= 0)
	{
		double x0 = vec.x.at (0);
		double z0 = vec.z.at (0);

		int i = 0;
		int N = vec.z.size () - 1;
		double distance;
		double height1, height2, delatHeight;
		while (i < N)
		{
			double x1 = vec.x.at (i);
			double z1 = vec.z.at (i);

			///double x2 = vec.x.at (i + 1);
			double z2 = vec.z.at (i + 1);

			height1 = z1 - z0;
			height2 = z2 - z0;
			delatHeight = std::fabs (z2 - z1);
			distance = (x1 - x0);

			if (height1 >= epsilonHeight && height2 >= epsilonHeight && delatHeight <= maxHeightDif && distance <= maxDistance)
			{
				ROS_INFO ("==>> Step detected appr. height: [%7.3f] and distance: [%7.3f], index1: [%3d]", vec.z.at (i), vec.x.at (i), i);

				return false;
			}
			i++;
		}
		return true;
	}
	else
	{
		ROS_INFO ("No Data available! Can't detect steps...");
		return false;
	}
}

/**
 * Detects a upward step
 * @param vec values of the laser scanner
 * @return true if step is detected
 */
int RobotControl::upwardStepDetection (DistanceVec vec)
{
	int N = vec.z.size () - 1;

	if (N >= 0)
	{
		double len = 0;
		double dis = 0;

		double x0 = vec.x.at (0);
		double z0 = vec.z.at (0);

		int i = 0;
		int N = vec.z.size () - 1;

		while (i < N)
		{
			double x1 = vec.x.at (i);
			double z1 = vec.z.at (i);

			double x2 = vec.x.at (i + 1);
			double z2 = vec.z.at (i + 1);

			len += std::hypot (x2 - x1, z2 - z1);
			dis = std::hypot (x2 - x0, z2 - z0);

			if ((std::fabs (len - dis) > step_height_treshold) && (std::fabs (x1) >= step_distance_treshold_min) && (std::fabs (x1) <= step_distance_treshold_max))

			{
				ROS_INFO ("==>> Step detected appr. height: [%7.3f] and distance: [%7.3f], index1: [%3d]", vec.z.at (i), vec.x.at (i), i);

				return 1;
			}
			i++;
		}
		return 0;
	}
	else
	{
		ROS_INFO ("No Data available! Can't detect steps...");
		return -1;
	}
}

/**
 * Drives a step upwards
 * @param distanceVecFront front values of the laser scanner
 * @param distanceVecRear rear values of the laser scanner
 * @param desiredVecFront values for the front flippers
 * @param desiredVecRear values for the rear flippers
 */
void RobotControl::upwardStepModeVec (DistanceVec distanceVecFront, DistanceVec distanceVecRear, VecXZPhi &desiredVecFront, VecXZPhi &desiredVecRear, double currentRearFlippperPossition, double currentFrontFlippperPossition)
{
	double zAtStepZ;
	double xAtStepZ;
	double phi;

	static double saveRearFlippperPossition;
	static double saveFrontFlippperPossition;

	snakeModeVec (distanceVecFront, desiredVecFront);
	if (desiredVecFront.phi < step_mode_front_angle_min)
	{
		desiredVecFront.phi = step_mode_front_angle_min;
	}
	switch (currentState)
	{
		case searchStep:
		{
			std::cout<<"searchStep"<<std::endl;
			snakeModeVec (distanceVecRear, desiredVecRear);
			if(upwardStepDetection (distanceVecFront)==1)
			{
				currentState=stepInitalisieren;
			}
			break;
		}
		case stepInitalisieren:
		{
			std::cout<<"stepInitalisieren"<<std::endl;
			std::vector<double>::iterator maxZIt;
			unsigned int maxZEl;
			double sign;
			double arg;
			maxZIt = std::max_element (distanceVecFront.z.begin (), distanceVecFront.z.end ());
			maxZEl = std::distance (distanceVecFront.z.begin (), maxZIt);
			stepZ = distanceVecFront.z.at (maxZEl);
			sign = distanceVecFront.x.at (0) / sqrt (distanceVecFront.x.at (0) * distanceVecFront.x.at (0));
			arg = sign * stepZ / (L - R);
			stepBeta = R2D * asin (arg);
			stepZ = stepZ + step_offset;
			saveFrontFlippperPossition=0;
			saveRearFlippperPossition=0;
			currentState=approachStep;
			break;
		}
		case approachStep:
		{
			std::cout<<"approachStep"<<std::endl;
			//  Roboter steht auf der Kante, konstanter Wert für Frontflipper
			if (std::fabs (bodyAngle) >= body_angle_treshold)
			{
				desiredVecFront.phi = flipper_front_angle_step_climb;
			}

			// Hintere Flipper halten Bodenkontakt bis Robterwinkel gleich dem Stufenwinkel
			snakeModeVec (distanceVecRear, desiredVecRear);
			if (std::fabs (bodyAngle) >= std::fabs (stepBeta) / 2.0)
			{
				setVelocityLimitation (true);
			}
			if ((std::fabs (bodyAngle) > (std::fabs (stepBeta) - bodyAngleThreshold)))
			{
				currentState=moveRearFlipper;
			}
			break;
		}
		case moveRearFlipper:
		{
			std::cout<<"moveRearFlipper"<<std::endl;
			double arg;
			double sign;
			arg = stepZ / (L - (R * 2.0));
			sign = distanceVecRear.x.at (0) / sqrt (distanceVecRear.x.at (0) * distanceVecRear.x.at (0));
			xAtStepZ = sign * stepZ * cos (arg) / sin (arg);
			zAtStepZ = -1.0 * (R + d);
			phi = snakeModeAngle (zAtStepZ);
			desiredVecRear.z = zAtStepZ;
			desiredVecRear.x = xAtStepZ;
			desiredVecRear.phi = phi;
			double zFront = 0.0;
			double xFront = -sign * (l - R);
			double phiFront = snakeModeAngle (zFront);
			desiredVecFront.z = zFront;
			desiredVecFront.x = xFront;
			desiredVecFront.phi = phiFront;
			saveFrontFlippperPossition=-currentFrontFlippperPossition;
			saveRearFlippperPossition=-currentRearFlippperPossition;
			//snakeModeVec (distanceVecFront, desiredVecFront);
			if (fabs(robotDirection*bodyAngle) <= step_angle_treshold)
			{
				std::cout<<"saveFrontFlippperPossition"<<saveFrontFlippperPossition<<std::endl;
				std::cout<<"saveRearFlippperPossition"<<saveRearFlippperPossition<<std::endl;
				currentState=waitForRearUSS;
			}
			break;
		}
		case waitForRearUSS:
		{
			zAtStepZ = NAN;
			xAtStepZ = NAN;
			std::cout<<"saveRearFlippperPossition"<<saveRearFlippperPossition<<std::endl;
			phi = saveRearFlippperPossition*R2D;
			desiredVecRear.z = zAtStepZ;
			desiredVecRear.x = xAtStepZ;
			desiredVecRear.phi = phi;
			std::cout<<"desiredVecRear.phi"<<desiredVecRear.phi<<std::endl;
			std::cout<<"saveFrontFlippperPossition"<<saveFrontFlippperPossition<<std::endl;
			desiredVecFront.phi = saveFrontFlippperPossition*R2D;
			std::cout<<"desiredVecFront.phi"<<desiredVecFront.phi<<std::endl;

			if (distanceVecRear.vd < step_rear_height_treshold)
			{
				currentState=setRearFlipperHorizontal;

			}
			break;
		}
		case setRearFlipperHorizontal:
		{
			std::cout<<"setRearFlipperHorizontal"<<std::endl;
			std::vector<double>::iterator maxZIt;
			unsigned int maxZEl;
			maxZIt = std::max_element (distanceVecRear.z.begin (), distanceVecRear.z.end ());
			maxZEl = std::distance (distanceVecRear.z.begin (), maxZIt);
			double rearHeight = distanceVecRear.z.at (maxZEl);
			double sign;
			sign = distanceVecRear.x.at (0) / sqrt (distanceVecRear.x.at (0) * distanceVecRear.x.at (0));
			zAtStepZ = 0.0;
			xAtStepZ = sign * (l - R);
			phi = snakeModeAngle (zAtStepZ);
			desiredVecRear.z = zAtStepZ;
			desiredVecRear.x = xAtStepZ;
			desiredVecRear.phi = phi;
			setVelocityLimitation (false);
			if (fabs(rearHeight) < 0.04)
			{
				currentState=stepOvercome;
			}
			break;
		}
		case stepOvercome:
		{
			std::cout<<"stepOvercome"<<std::endl;
			stepReset ();
			snakeModeVec (distanceVecRear, desiredVecRear);
			break;
		}
		default:
		{
			double Mid1_ussDistance = 0.10;
			double Mid2_ussDistance = 0.10;


			if (flipperMode != 0 && frontDesiredAngle <= angleThreshold && mid1_uss <= Mid1_ussDistance)
				frontDesiredAngle = NAN;
			if (flipperMode != 0 && rearDesiredAngle <= angleThreshold && mid2_uss <= Mid2_ussDistance)
				rearDesiredAngle = NAN;
			break;
		}
	}
}

/**
 * Does a reset after a step was climbed
 */
void RobotControl::stepReset ()
{
	setVelocityLimitation (false);
	stepDetected = -1;
	stepClimb = false;
	stepClimbComplete = false;
	stepBorderReached = false;
	stepInit = false;
	stepBeta = 0.0;
	stepZ = 0.0;
	stateStep = RESET;
	previousStateStep = stateStep;
	currentState=searchStep;
}

/**
 * Sets a new velocity limitation
 * @param state new limitation value
 */
void RobotControl::setVelocityLimitation (bool state)
{
	velocityLimitationOld = velocityLimitation;
	velocityLimitation = state;
}


void RobotControl::publishVelocities ()
{
	geometry_msgs::Twist velocities;
	if (moveRobot == true)
	{

		if (selectDriveForward == true)
		{
			velocities.angular.z = 0;
			velocities.linear.x = 0.05;
			std::cout << "selectDriveForward" << selectDriveForward << std::endl;
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
	velPub.publish (velocities);
}
void RobotControl::publishAngles ()
{
	if (frontDesiredAngle != NAN)
	{
		frontFlipperAngleDesiredMsg.data = -frontDesiredAngle * D2R;
	}
	else
		ROS_INFO (" Front angle NAN:\t  [%7.3f]", frontDesiredAngle);
	if (rearDesiredAngle != NAN)
	{
		rearFlipperAngleDesiredMsg.data = -rearDesiredAngle * D2R;
	}
	else
		ROS_INFO (" Rear angle NAN:\t  [%7.3f]", rearDesiredAngle);

	if(frontFlipperAngleDesiredMsg.data<=M_PI/2 && frontFlipperAngleDesiredMsg.data>=-M_PI/2)
	{
		std::cout<<"frontFlipperAngleDesiredMsg.data"<<frontFlipperAngleDesiredMsg.data<<std::endl;
		frontFlipperAngleDesiredPub.publish (frontFlipperAngleDesiredMsg);
	}

	if(rearFlipperAngleDesiredMsg.data<=M_PI/2 && rearFlipperAngleDesiredMsg.data>=-M_PI/2)
	{
		std::cout<<"rearFlipperAngleDesiredMsg.data"<<rearFlipperAngleDesiredMsg.data<<std::endl;
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

	double step_mode_front_angle_min_old = step_mode_front_angle_min;
	double snake_mode_front_angle_min_old = snake_mode_front_angle_min;
	double snake_mode_rear_angle_min_old = snake_mode_rear_angle_min;
	double body_angle_treshold_old = body_angle_treshold;
	double step_offset_old = step_offset;
	double body_front_0_distance_offset_old = body_front_0_distance_offset;
	double body_front_1_distance_offset_old = body_front_1_distance_offset;
	double body_rear_0_distance_offset_old = body_rear_0_distance_offset;
	double body_rear_1_distance_offset_old = body_rear_1_distance_offset;
	double step_angle_treshold_old = step_angle_treshold;
	double step_rear_height_treshold_old = step_rear_height_treshold;
	double step_height_treshold_old = step_height_treshold;
	double step_distance_treshold_min_old = step_distance_treshold_min;
	double step_distance_treshold_max_old = step_distance_treshold_max;
	double flipper_front_angle_step_climb_old = flipper_front_angle_step_climb;
	double flipper_angle_min_old = flipper_angle_min;
	double flipper_angle_max_old = flipper_angle_max;
	double laser_scan_front_angle_min_old = laser_scan_front_angle_min;
	double laser_scan_front_angle_max_old = laser_scan_front_angle_max;
	double laser_scan_rear_angle_min_old = laser_scan_rear_angle_min;
	double laser_scan_rear_angle_max_old = laser_scan_rear_angle_max;
	double Hoku_scan_angle_min_old = Hoku_scan_angle_min;
	double Hoku_scan_angle_max_old = Hoku_scan_angle_max;
	startStairBehavior= confg.startStairBehavior;
	step_mode_front_angle_min = confg.step_mode_front_angle_min;
	snake_mode_front_angle_min = confg.snake_mode_front_angle_min;
	snake_mode_rear_angle_min = confg.snake_mode_rear_angle_min;
	body_angle_treshold = confg.body_angle_treshold;
	step_offset = confg.step_offset;
	body_front_0_distance_offset = confg.body_front_0_distance_offset;
	body_front_1_distance_offset = confg.body_front_1_distance_offset;
	body_rear_0_distance_offset = confg.body_rear_0_distance_offset;
	body_rear_1_distance_offset = confg.body_rear_1_distance_offset;
	step_angle_treshold = confg.step_angle_treshold;
	step_rear_height_treshold = confg.step_rear_height_treshold;
	step_height_treshold = confg.step_height_treshold;
	step_distance_treshold_min = confg.step_distance_treshold_min;
	step_distance_treshold_max = confg.step_distance_treshold_max;
	flipper_front_angle_step_climb = confg.flipper_front_angle_step_climb;

	flipper_angle_min = confg.flipper_angle_min;
	flipper_angle_max = confg.flipper_angle_max;
	laser_scan_front_angle_min = D2R * confg.laser_scan_front_angle_min;
	laser_scan_front_angle_max = D2R * confg.laser_scan_front_angle_max;
	laser_scan_rear_angle_min = D2R * confg.laser_scan_rear_angle_min;
	laser_scan_rear_angle_max = D2R * confg.laser_scan_rear_angle_max;
	Hoku_scan_angle_min = D2R * confg.Hoku_scan_angle_min;
	Hoku_scan_angle_max = D2R * confg.Hoku_scan_angle_max;

	if (std::fabs (step_mode_front_angle_min_old - step_mode_front_angle_min) != 0.0)
		ROS_INFO ("Step mode min. front angle changed:  from [%7.3f] to [%7.3f]", step_mode_front_angle_min_old, step_mode_front_angle_min);

	if (std::fabs (snake_mode_front_angle_min_old - snake_mode_front_angle_min) != 0.0)
		ROS_INFO ("Snake mode min. front angle changed: from [%7.3f] to [%7.3f]", snake_mode_front_angle_min_old, snake_mode_front_angle_min);

	if (std::fabs (snake_mode_rear_angle_min_old - snake_mode_rear_angle_min) != 0.0)
		ROS_INFO ("Snake mode min. rear angle changed:  from [%7.3f] to [%7.3f]", snake_mode_rear_angle_min_old, snake_mode_rear_angle_min);

	if (std::fabs (body_angle_treshold_old - body_angle_treshold) != 0.0)
		ROS_INFO ("Body angle treshold changed:         from [%7.3f] to [%7.3f]", body_angle_treshold_old, body_angle_treshold);

	if (std::fabs (step_offset_old - step_offset) != 0.0)
		ROS_INFO ("Step offset changed:                 from [%7.3f] to [%7.3f]", step_offset_old, step_offset);

	if (std::fabs (body_front_0_distance_offset_old - body_front_0_distance_offset) != 0.0)
		ROS_INFO ("Body front0 distance offset changed: from [%7.3f] to [%7.3f]", body_front_0_distance_offset_old, body_front_0_distance_offset);
	else
		ROS_INFO ("Body front0 distance offset:              [%7.3f]", body_front_0_distance_offset);

	if (std::fabs (body_front_1_distance_offset_old - body_front_1_distance_offset) != 0.0)
		ROS_INFO ("Body front1 distance offset changed: from [%7.3f] to [%7.3f]", body_front_1_distance_offset_old, body_front_1_distance_offset);
	else
		ROS_INFO ("Body front1 distance offset:              [%7.3f]", body_front_1_distance_offset);

	if (std::fabs (body_rear_0_distance_offset_old - body_rear_0_distance_offset) != 0.0)
		ROS_INFO ("Body rear0 distance offset changed:  from [%7.3f] to [%7.3f]", body_rear_0_distance_offset_old, body_rear_0_distance_offset);
	else
		ROS_INFO ("Body rear0 distance offset:               [%7.3f]", body_rear_0_distance_offset);

	if (std::fabs (body_rear_1_distance_offset_old - body_rear_1_distance_offset) != 0.0)
		ROS_INFO ("Body rear1 distance offset changed:  from [%7.3f] to [%7.3f]", body_rear_1_distance_offset_old, body_rear_1_distance_offset);
	else
		ROS_INFO ("Body rear1 distance offset:               [%7.3f]", body_rear_1_distance_offset);

	if (std::fabs (step_angle_treshold_old - step_angle_treshold) != 0.0)
		ROS_INFO ("Step angle treshold changed:         from [%7.3f] to [%7.3f]", step_angle_treshold_old, step_angle_treshold);
	else
		ROS_INFO ("Step angle treshold:                      [%7.3f]", step_angle_treshold);

	if (std::fabs (step_rear_height_treshold_old - step_rear_height_treshold) != 0.0)
		ROS_INFO ("Step rear height treshold changed:   from [%7.3f] to [%7.3f]", step_rear_height_treshold_old, step_rear_height_treshold);

	if (std::fabs (step_height_treshold_old - step_height_treshold) != 0.0)
		ROS_INFO ("Step height treshold changed:        from [%7.3f] to [%7.3f]", step_height_treshold_old, step_height_treshold);

	if (std::fabs (step_distance_treshold_min_old - step_distance_treshold_min) != 0.0)
		ROS_INFO ("Step min distance treshold changed:  from [%7.3f] to [%7.3f]", step_distance_treshold_min_old, step_distance_treshold_min);

	if (std::fabs (step_distance_treshold_max_old - step_distance_treshold_max) != 0.0)
		ROS_INFO ("Step max distance treshold changed:  from [%7.3f] to [%7.3f]", step_distance_treshold_max_old, step_distance_treshold_max);

	if (std::fabs (flipper_front_angle_step_climb_old - flipper_front_angle_step_climb_old) != 0.0)
		ROS_INFO ("Flipper max angle step up changed:   from [%7.3f] to [%7.3f]", flipper_front_angle_step_climb_old, flipper_front_angle_step_climb);

	if (std::fabs (flipper_angle_min_old - flipper_angle_min) != 0.0)
		ROS_INFO ("Flipper min angle changed:           from [%7.3f] to [%7.3f]", flipper_angle_min_old, flipper_angle_min);

	if (std::fabs (flipper_angle_max_old - flipper_angle_max) != 0.0)
		ROS_INFO ("Flipper max angle changed:           from [%7.3f] to [%7.3f]", flipper_angle_max_old, flipper_angle_max);

	if (std::fabs (laser_scan_front_angle_min_old - laser_scan_front_angle_min) != 0.0)
		ROS_INFO ("Laser scan front min angle changed:  from [%7.3f] to [%7.3f]", laser_scan_front_angle_min_old, laser_scan_front_angle_min);

	if (std::fabs (laser_scan_front_angle_max_old - laser_scan_front_angle_max) != 0.0)
		ROS_INFO ("Laser scan front max angle changed:  from [%7.3f] to [%7.3f]", laser_scan_front_angle_max_old, laser_scan_front_angle_max);

	if (std::fabs (laser_scan_rear_angle_min_old - laser_scan_rear_angle_min) != 0.0)
		ROS_INFO ("Laser scan rear min angle changed:   from [%7.3f] to [%7.3f]", laser_scan_rear_angle_min_old, laser_scan_rear_angle_min);

	if (std::fabs (laser_scan_rear_angle_max_old - laser_scan_rear_angle_max) != 0.0)
		ROS_INFO ("Laser scan rear max angle changed:   from [%7.3f] to [%7.3f]\n", laser_scan_rear_angle_max_old, laser_scan_rear_angle_max);

	if (std::fabs (laser_scan_front_angle_min_old - laser_scan_front_angle_min) != 0.0)
		ROS_INFO ("Hoku scan min angle changed:  from [%7.3f] to [%7.3f]", Hoku_scan_angle_min_old, Hoku_scan_angle_min);

	if (std::fabs (laser_scan_front_angle_max_old - laser_scan_front_angle_max) != 0.0)
		ROS_INFO ("Hoku scan max angle changed:  from [%7.3f] to [%7.3f]", Hoku_scan_angle_max_old, Hoku_scan_angle_max);
	if (std::fabs (laser_scan_front_angle_min_old - laser_scan_front_angle_min) != 0.0)

		ROS_INFO ("******************************************************************************************************\n \n");
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
