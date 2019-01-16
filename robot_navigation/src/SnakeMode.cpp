/*
 * SnakeMode.cpp
 *
 *  Created on: 20.05.2018
 *      Author: chfo
 */



#include "SnakeMode.h"


constexpr double D2R = M_PI / 180.0;
constexpr double R2D = 180.0 / M_PI;


constexpr double DEGREE_20 = 80.0;
constexpr double DEGREE_110  = 440.0;
constexpr double DEGREE_200 = 800.0;
SnakeMode::SnakeMode ()
{
	currentTime = ros::Time::now ().toSec ();
	previousTime = currentTime;
	front_uss=0;
	mid1_uss=0;
	mid2_uss=0;
	rear_uss=0;

	R = 0.08;
	r = 0.0375;
	l = 0.22;
	d = std::hypot (r, l);
	theta = atan (r / l);
	flipper_angle_min=-90;
	flipper_angle_max=70;

}
SnakeMode::~SnakeMode ()
{

}
void SnakeMode::SetUss (double front, double mid1, double mid2, double rear)
{
	front_uss=front;
	mid1_uss=mid1;
	mid2_uss=mid2;
	rear_uss=rear;
}

void SnakeMode::DistVect (DistanceVec frontDist, DistanceVec rearDist)
{
	front=frontDist;
	rear=rearDist;
}

void SnakeMode::SetSnakeModeParams (double flipperAngleMax, double flipperAngleMin)
{
	flipper_angle_min=flipperAngleMin;
	flipper_angle_max=flipperAngleMax;
}


void SnakeMode::snakeMode (double &frontDesiredAngle,double &rearDesiredAngle)
{
	std::cout << "---------snakeMode-------- " << std::endl;
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
	desiredVecRear.phi = (desiredVecRear.phi > -25.0) ? desiredVecRear.phi : -25.0;


	frontDesiredAngle = desiredVecFront.phi;
	rearDesiredAngle = desiredVecRear.phi;
	double distanceUltrasoundSensor = 0.33;
	//double bodyAngleUtrasoundSensor = -atan2 (mid1_uss - mid2_uss, distanceUltrasoundSensor);
}


/**
 * Gets the height and distance of the next obstacle
 * @param distanceVec values of the laser scanner
 * @param desiredVec values of the next obstacle and desired flipper angle
 */
void SnakeMode::snakeModeVec (DistanceVec distanceVec, VecXZPhi &desiredVec)
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

void SnakeMode::snakeModeVecAbs(DistanceVec distanceVec, VecXZPhi &desiredVec)
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
	desiredVec.phi = fabs(phi);

}

double SnakeMode::snakeModeAngle (double z)
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

