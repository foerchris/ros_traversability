/*
 * StairBehavior.cpp
 *
 *  Created on: 13.05.2018
 *      Author: chfo
 */


#include "StairBehavior.h"


constexpr double D2R = M_PI / 180.0;
constexpr double R2D = 180.0 / M_PI;


constexpr double DEGREE_20 = 80.0;
constexpr double DEGREE_110  = 440.0;
constexpr double DEGREE_200 = 800.0;
StairBehavior::StairBehavior ()
{
	currentTime = ros::Time::now ().toSec ();
	previousTime = currentTime;
	moveRobot = true;
	bodyAngleRoll=0;
	bodyAnglePitch=0;
	bodyAngle=0;
	front_uss=0;
	mid1_uss=0;
	mid2_uss=0;
	rear_uss=0;

	stairUpStat= start;
	stairDownStat= start;
	// dynamic reconfigure
	static dynamic_reconfigure::Server<robot_navigation::StairBehaviorConfig> serverStairCon (ros::NodeHandle ("~/StairBehavior"));
	static dynamic_reconfigure::Server<robot_navigation::StairBehaviorConfig>::CallbackType f2;
	f2 = boost::bind(&StairBehavior::reconfigureCallback,this,_1,_2);
	serverStairCon.setCallback(f2);
}
StairBehavior::~StairBehavior ()
{

}
/**
 * Mode for driving a stair down
 * @param stairAngle slope angle of the stair
 * @param stepHeight height of the first step edge
 */

void StairBehavior::SetStairBehaviorParams (double flipperAngleMax, double flipperAngleMin)
{
	snakeMode.SetSnakeModeParams(flipperAngleMax,flipperAngleMin);
}


void StairBehavior::SetBodyAngle (double pitch, double roll, double angle)
{
	bodyAnglePitch = pitch;
	bodyAngleRoll = roll;
	bodyAngle = angle;
}
void StairBehavior::MoveRobot (bool& move)
{
	move=moveRobot;
}
void StairBehavior::SetUss (double front, double mid1, double mid2, double rear)
{
	front_uss=front;
	mid1_uss=mid1;
	mid2_uss=mid2;
	rear_uss=rear;
	snakeMode.SetUss(front_uss,mid1_uss,mid2_uss,rear_uss);
}

void StairBehavior::DistVect (DistanceVec frontDist, DistanceVec rearDist,DistanceVec frontRotBodyAngle, DistanceVec rearRotBodyAngle)
{
	front=frontDist;
	rear=rearDist;
	front_rotBodyAngle=frontRotBodyAngle;
	rear_rotBodyAngle=rearRotBodyAngle;
	snakeMode.DistVect(front,rear);

}

/**
 * Mode to drive a stair up
 * @param stairAngle slope angle of the stair
 * @param stepHeight height of the first step edge
 */
void StairBehavior::stairUpMode (double stairAngle, double stepHeight,double &frontDesiredAngle,double &rearDesiredAngle)
{
	std::cout << "---------stairUpMode-------- " << std::endl;
	double deltaSubtrackAngle = -11;
	double distanceUssMin = 0.10;
	moveRobot = true;

	double distanceUltrasoundSensor = 0.33;

	///double distanceUssMidThreshold = 0.10; // Schwellwert  zum Boden
	///double distanceRearThreshold = 0.30; // Schwellwert nach  Hinten
	///double bodyAngThreshold = 3.0;
	double bodyAngleUtrasoundSensor = -atan2 (mid1_uss - mid2_uss, distanceUltrasoundSensor);
	///static bool upWards = false;

	VecXZPhi desiredVecFront, desiredVecRear;


	double epsilonAngle = 3.0 * D2R;
	///double friction = 0.8;
	double marginAngle = 10 * D2R;
	double deltaAngle = std::fabs (std::fabs (bodyAnglePitch) - stairAngle);
	double deltaAngle2 =fabs( fabs(bodyAnglePitch) - stairAngle);
	double deltaAngleOffset = fabs(bodyAnglePitch) - stairAngle - marginAngle;
	printf ("bodyAnglePitch: %4.4f  stairAngle: %4.4f  marginAngle: %4.4f \n", bodyAnglePitch, stairAngle, marginAngle);

	std::cout << "deltaAngleOffset: " << deltaAngleOffset * R2D << std::endl;

	///double desiredAngularVel;
	///double deltadirectionAngle;
	///double controlGain;
	///double stairAngleNDeg = -stairAngle * R2D;
	double Kp = 1; //0.03
	double Ki = 0.8; //10
	double Kd = 0.1; // 2
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
	/*std::cout << "deltaAngle2: " << deltaAngle2 * R2D << std::endl;
	std::cout << "epsilonAngle: " << epsilonAngle * R2D << std::endl;
	std::cout << "bodyAnglePitch: " << bodyAnglePitch * R2D << std::endl;
	std::cout << "previousTime: " << previousTime << std::endl;
	std::cout << "currentTime: " << currentTime << std::endl;
	if (front_uss <= distanceUssMin && rear_uss <= distanceUssMin && fabs (bodyAngle) <= epsilonAngle * R2D)
	{
		stairUpStat = start;
	}
	int sizeOfVec = front_rotBodyAngle.z.size ();
	std::cout<<"sizeOfVec"<<sizeOfVec<<std::endl;
	for (int row = 0; row < sizeOfVec - 1; row++)
	{
		std::cout<<"front_rotBodyAngle.z.at (row)"<<	front_rotBodyAngle.z.at (row)<<std::endl;
	}
	sizeOfVec = front.z.size ();
	std::cout<<"sizeOfVec"<<sizeOfVec<<std::endl;

	for (int row = 0; row < sizeOfVec - 1; row++)
	{
		std::cout<<"front.z.at (row)"<<	front.z.at (row)<<std::endl;
	}*/
	switch (stairUpStat)
	{
		case start:
			std::cout << "*********************start*********************"<< std::endl;

			//desiredVecRear.phi = snakeMode.snakeModeAngle (stepHeight);
			//desiredVecFront.phi = bodyAngle + deltaSubtrackAngle + marginAngle;
			desiredVecFront.phi = snakeMode.snakeModeAngle (stepHeight);
			desiredVecRear.phi = fabs(-fabs (bodyAngle) + deltaSubtrackAngle + marginAngle);

			if (deltaAngle >= epsilonAngle && fabs (bodyAnglePitch) >= epsilonAngle)
			{
				stairUpStat = ptichUp;
			}
			break;

		case ptichUp:
			std::cout << "*********************ptichUp*********************"<< std::endl;

			desiredVecFront.phi = snakeMode.snakeModeAngle (stepHeight)-fabs(-fabs (bodyAngle) + deltaSubtrackAngle + marginAngle);
			desiredVecRear.phi = fabs(-fabs (bodyAngle) + deltaSubtrackAngle + marginAngle);

			/*desiredVecFront.phi = bodyAngle + deltaSubtrackAngle + marginAngle;
			desiredVecRear.phi = snakeMode.snakeModeAngle (stepHeight);*/
			if (deltaAngle <= epsilonAngle)
			{
				stairUpStat = normalClimbing;
			}
			break;

		case normalClimbing:
			std::cout << "*********************normalClimbing*********************"<< std::endl;

			desiredVecFront.phi = deltaSubtrackAngle;
			desiredVecRear.phi = deltaSubtrackAngle;


			endOfStair = upwardEndOfStairDetection (front_rotBodyAngle);
			std::cout << "upwardEndOfStairDetection (front_rotBodyAngle): " << endOfStair << std::endl;
			if ((endOfStair == true && (rear_uss <= distanceUssMin || mid2_uss <= distanceUssMin)))
			{
				stairUpStat = finalClimbing;
				moveRobot = false;
				lastErrorTerm = -3;
				integral = -3;
				phi = deltaSubtrackAngle;
				previousTime=currentTime;
			}
			if (endOfStair == true && deltaAngle >= epsilonAngle)
			{
				stairUpStat = reset;
				moveRobot = false;
				lastErrorTerm = 0.0;
				integral = 0.0;
				previousTime=currentTime;
				phi = deltaSubtrackAngle;
			}
			break;

		case finalClimbing:
			std::cout << "*********************finalClimbing*********************"<< std::endl;
			dt = (currentTime - previousTime);
			errorTerm = deltaAngleOffset * R2D;
			integral += errorTerm * dt;
			pidAngle = Kp * errorTerm + Ki * integral + Kd * (errorTerm - lastErrorTerm) / dt;
			//printf ("dt: %4.4f  integral: %4.4f  marginAngle: %4.4f lastErrorTerm: %4.4f \n", dt, integral, errorTerm,lastErrorTerm);

			lastErrorTerm = errorTerm;
			previousTime = currentTime;
			pidAngle = (pidAngle < 0.0) ? pidAngle : 0.0;
			pidAngle = (pidAngle > -60.0) ? pidAngle : -60.0;
			phi = pidAngle;
			std::cout << "phi: " << phi << std::endl;

			desiredVecFront.phi = phi;
			desiredVecRear.phi = deltaSubtrackAngle;


			std::cout << "deltaAngle2: " << deltaAngle2 * R2D << " epsilonAngle: " << epsilonAngle * R2D << std::endl;

			if (fabs(deltaAngle2) >= epsilonAngle)
			{
				stairUpStat = pitchDown;
				previousTime = currentTime;
				lastErrorTerm = 0.0;
				integral = 0.0;
			}
			break;

		case pitchDown:
			std::cout << "*********************pitchDown*********************"<< std::endl;

			desiredVecFront.phi = phi;
			desiredVecRear.phi = deltaSubtrackAngle;

			std::cout << "0.8 * stairAngle: " << 0.8 * stairAngle << std::endl;
			std::cout << "bodyAnglePitch: " << bodyAnglePitch << std::endl;
			if (front_uss <= distanceUssMin && bodyAnglePitch <= (0.8 * stairAngle))
			{
				stairUpStat = reset;
				previousTime = currentTime;
			}
			break;

		case reset:
			std::cout << "*********************reset*********************"<< std::endl;

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

			desiredVecRear.phi = bodyAngle + deltaSubtrackAngle;
			desiredVecFront.phi = phi;
			if (fabs (bodyAnglePitch) <= epsilonAngle)
			{
				stairUpStat = start;
			}
			if (front_uss <= distanceUssMin && bodyAnglePitch <= (0.7 * stairAngle))
				stairUpStat = start;
			break;
		default:
			break;
	}
	frontDesiredAngle = desiredVecFront.phi;
	rearDesiredAngle = desiredVecRear.phi;

	//ROS_INFO (" distancePitch %f", distancePitch);
	ROS_INFO (" stairAngle %f", stairAngle * R2D);
	ROS_INFO (" bodyAngleRoll %f", bodyAngleRoll * R2D);
	ROS_INFO (" bodyAngle: [%3.3f] \t\t bodyAngleUtrasoundSensor: [%3.3f]", bodyAngle, bodyAngleUtrasoundSensor * R2D);
	ROS_INFO (" stairUpStat %i", stairUpStat);
	ROS_INFO (" Front flipper:\t desired angle: [%7.3f]", desiredVecFront.phi);
	ROS_INFO ("  Rear flipper:\t desired angle: [%7.3f]", desiredVecRear.phi);
}


void StairBehavior::stairDownMode (double stairAngle, double stepHeight,double &frontDesiredAngle,double &rearDesiredAngle)
{

	std::cout << "---------stairDownMode-------- " << std::endl;

	double deltaSubtrackAngle = 0.0;
	double distanceUssMax = 0.20;
	double distanceUssMin = 0.10;
	double distanceUssMidThreshold = 0.10; // distance to ground

	VecXZPhi desiredVecFront, desiredVecRear;


	double epsilonAngle = 3.0 * D2R;
	double deltaAngleAbs = std::fabs (bodyAnglePitch - stairAngle);
	double deltaAngle = stairAngle - bodyAnglePitch;
	double marginAngle = 5.0;

	double flipper_angle_max_temp = 45;
	double flipper_angle_min_temp = -90;

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
		stairDownStat = start;
	}
	switch (stairDownStat)
	{
		case start:
			std::cout << "stairDownStat"<< std::endl;
			snakeMode.snakeModeVec (rear, desiredVecRear);
			desiredVecFront.phi = deltaSubtrackAngle;
			if (front_uss >= distanceUssMin)
			{
				moveRobot = false;
				stairDownStat = ptichUp;
				phi = 0;
			}
			break;

		case ptichUp:
			std::cout << "ptichUp"<< std::endl;

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
				stairDownStat = startClimbing;

			}
			break;

		case startClimbing:
			std::cout << "ptichUp"<< std::endl;

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
				stairDownStat = normalClimbing;
			}
			break;

		case normalClimbing:
			std::cout << "normalClimbing"<< std::endl;

			desiredVecFront.phi = deltaSubtrackAngle;
			desiredVecRear.phi = deltaSubtrackAngle;

			endOfStair = upwardEndOfStairDetection (front_rotBodyAngle);
			std::cout << "upwardEndOfStairDetection (front_rotBodyAngle): " << endOfStair << std::endl;
			if (endOfStair == true && front_uss <= distanceUssMin)
			{
				stairDownStat = finalClimbing;
			}
			break;
		case finalClimbing:
			std::cout << "finalClimbing"<< std::endl;

			desiredVecFront.phi = deltaSubtrackAngle + 2 * marginAngle;
			desiredVecRear.phi = deltaSubtrackAngle - marginAngle;

			if (deltaAngle >= (2 * epsilonAngle) && front_uss >= distanceUssMidThreshold)
			{
				stairDownStat = pitchDown;
				moveRobot = false;
				previousTime = currentTime;
			}
			break;

		case pitchDown:
			std::cout << "pitchDown"<< std::endl;

			desiredVecFront.phi = deltaSubtrackAngle;
			snakeMode.snakeModeVec (rear, desiredVecRear);
			if ((currentTime - previousTime) <= 1.0)
				moveRobot = false;
			if (rear_uss <= distanceUssMin && (currentTime - previousTime) >= 1.0)
			{
				stairDownStat = reset;
			}
			break;

		case reset:
			std::cout << "reset"<< std::endl;

			desiredVecFront.phi = bodyAngle + deltaSubtrackAngle + marginAngle;
			snakeMode.snakeModeVec (rear, desiredVecRear);

			if (fabs (bodyAnglePitch) <= epsilonAngle)
			{
				stairDownStat = start;
			}
			break;
		default:
			break;
	}
	frontDesiredAngle = desiredVecFront.phi;
	rearDesiredAngle = desiredVecRear.phi;
	ROS_INFO (" stairAngle %f", stairAngle * R2D);
	ROS_INFO (" bodyAngleRoll %f", bodyAngleRoll * R2D);
	ROS_INFO (" stairDownStat %i", stairDownStat);
	ROS_INFO (" Front flipper:\t desired angle: [%7.3f]", desiredVecFront.phi);
	ROS_INFO ("  Rear flipper:\t desired angle: [%7.3f]", desiredVecRear.phi);

}

bool StairBehavior::upwardEndOfStairDetection (DistanceVec vec)
{

	int N = vec.z.size () - 1;

	double stepHeight = vec.vd;
	std::cout<<"stepHeight"<<stepHeight<<std::endl;
	double epsilonHeight = 0.5 * stepHeight;
	double maxHeightDif = 0.03;
	double maxDistance = 0.8;
	if (N >= 0)
	{
		double x0 = fabs(vec.x.at (0));
		double z0 = vec.z.at (0);

		int i = 0;
		int N = vec.z.size () - 1;
		double distance;
		double height1, height2, delatHeight;
		while (i < N)
		{
			double x1 = fabs(vec.x.at (i));
			double z1 = vec.z.at (i);

			std::cout<<"x0: "<<x0<<std::endl;
			std::cout<<"z0: "<<z0<<std::endl;

			///double x2 = vec.x.at (i + 1);
			double z2 = vec.z.at (i + 1);
			std::cout<<"x1: "<<x1<<std::endl;
			std::cout<<"z1: "<<z1<<std::endl;
			std::cout<<"z2: "<<z2<<std::endl;

			height1 = z1 - z0;
			height2 = z2 - z0;
			std::cout<<"height1: "<<height1<<std::endl;
			std::cout<<"height2: "<<height2<<std::endl;

			delatHeight = std::fabs (z2 - z1);
			std::cout<<"delatHeight: "<<delatHeight<<std::endl;

			distance = (x1 - x0);
			std::cout<<"distance: "<<distance<<std::endl;

			if (height1 >= epsilonHeight && height2 >= epsilonHeight && delatHeight <= maxHeightDif && distance <= maxDistance)
			{
				std::cout<<"==>> Step detected appr. height: " <<vec.z.at (i)<<" and distance: "<< vec.x.at (i)<<"index1: "<< i<<std::endl;

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

void StairBehavior::reconfigureCallback (robot_navigation::StairBehaviorConfig &confg, uint32_t level)
{
	ROS_INFO ("************************************ Reconfigure callback ********************************************\n");


}

