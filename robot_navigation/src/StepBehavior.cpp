/*
 * StepBehavior.cpp
 *
 *  Created on: 21.05.2018
 *      Author: chfo
 */


#include "robot_navigation/StepBehavior.h"


constexpr double D2R = M_PI / 180.0;
constexpr double R2D = 180.0 / M_PI;


constexpr double DEGREE_20 = 80.0;
constexpr double DEGREE_110  = 440.0;
constexpr double DEGREE_200 = 800.0;

StepBehavior::StepBehavior ()
{
	L = 0.48;
	R = 0.08;
	l = 0.22;
	r = 0.0375;
	d = std::hypot (r, l);




	bodyAngleThreshold = 6.0;


	bodyAngleRoll=0;
	bodyAnglePitch=0;
	bodyAngle=0;
	front_uss=0;
	mid1_uss=0;
	mid2_uss=0;
	rear_uss=0;
	stepBeta = 0.0;
	stepZ = 0.0;
	stateStep = START;
	currentState=searchStep;
	robotDirection = 0;

	// dynamic reconfigure
	static dynamic_reconfigure::Server<robot_navigation::StepBehaviorConfig> serverStepCon (ros::NodeHandle ("~/StepBehavior"));
	static dynamic_reconfigure::Server<robot_navigation::StepBehaviorConfig>::CallbackType f2;
	f2 = boost::bind(&StepBehavior::reconfigureCallback,this,_1,_2);
	serverStepCon.setCallback(f2);

}
StepBehavior::~StepBehavior ()
{

}
/**
 * Mode for driving a stair down
 * @param stairAngle slope angle of the stair
 * @param stepHeight height of the first step edge
 */

void StepBehavior::SetStepBehaviorParams (double flipperAngleMax, double flipperAngleMin)
{
	snakeMode.SetSnakeModeParams(flipperAngleMax,flipperAngleMin);
}


void StepBehavior::SetBodyAngle (double pitch, double roll, double angle)
{
	bodyAnglePitch = pitch;
	bodyAngleRoll = roll;
	bodyAngle = angle;
}

void StepBehavior::SetUss (double front, double mid1, double mid2, double rear)
{
	front_uss=front;
	mid1_uss=mid1;
	mid2_uss=mid2;
	rear_uss=rear;
	snakeMode.SetUss(front_uss,mid1_uss,mid2_uss,rear_uss);
}

void StepBehavior::DistVect (DistanceVec frontDist, DistanceVec rearDist, double currentFrontAngle, double currentRearAngle, int direction)
{
	front=frontDist;
	rear=rearDist;
	snakeMode.DistVect(front,rear);
	currentFlipperAngleFront=currentFrontAngle;
	currentFlipperAngleRear=currentRearAngle;
	robotDirection=direction;

}

/**
 * Mode to drive a step up in front or rear direction
 */
void StepBehavior::upwardStepMode (double& frontDesiredAngle, double& rearDesiredAngle)
{
	VecXZPhi desiredVecFront;
	VecXZPhi desiredVecRear;
	desiredVecFront.phi=NAN;
	desiredVecRear.phi=NAN;
	std::cout << "---------upwardStepMode-------- " << std::endl;
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
			snakeMode.snakeModeVec (front, desiredVecFront);
			snakeMode.snakeModeVec (rear, desiredVecRear);
		}

	}
	frontDesiredAngle = desiredVecFront.phi;
	rearDesiredAngle = desiredVecRear.phi;
}



/**
 * Drives a step upwards
 * @param distanceVecFront front values of the laser scanner
 * @param distanceVecRear rear values of the laser scanner
 * @param desiredVecFront values for the front flippers
 * @param desiredVecRear values for the rear flippers
 */
void StepBehavior::upwardStepModeVec (DistanceVec distanceVecFront, DistanceVec distanceVecRear, VecXZPhi &desiredVecFront, VecXZPhi &desiredVecRear, double currentRearFlippperPossition, double currentFrontFlippperPossition)
{
	double zAtStepZ;
	double xAtStepZ;
	double phi;

	static double saveRearFlippperPossition;
	static double saveFrontFlippperPossition;

	snakeMode.snakeModeVec (distanceVecFront, desiredVecFront);
	if (desiredVecFront.phi < step_mode_front_angle_min)
	{
		desiredVecFront.phi = step_mode_front_angle_min;
	}
	switch (currentState)
	{
		case searchStep:
		{
			std::cout<<"searchStep"<<std::endl;
			snakeMode.snakeModeVec (distanceVecRear, desiredVecRear);
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
			snakeMode.snakeModeVec (distanceVecRear, desiredVecRear);
			//if (std::fabs (bodyAngle) >= std::fabs (stepBeta) / 2.0)
			//{
			//	setVelocityLimitation (true);
			//}
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
			phi = snakeMode.snakeModeAngle (zAtStepZ);
			desiredVecRear.z = zAtStepZ;
			desiredVecRear.x = xAtStepZ;
			desiredVecRear.phi = phi;
			double zFront = 0.0;
			double xFront = -sign * (l - R);
			double phiFront = snakeMode.snakeModeAngle (zFront);
			desiredVecFront.z = zFront;
			desiredVecFront.x = xFront;
			desiredVecFront.phi = phiFront;
			saveFrontFlippperPossition=-currentFrontFlippperPossition;
			saveRearFlippperPossition=-currentRearFlippperPossition;
			if (fabs(robotDirection*bodyAngle) <= step_angle_treshold)
			{
				currentState=waitForRearUSS;
			}
			break;
		}
		case waitForRearUSS:
		{
			zAtStepZ = NAN;
			xAtStepZ = NAN;
			phi = saveRearFlippperPossition*R2D;
			desiredVecRear.z = zAtStepZ;
			desiredVecRear.x = xAtStepZ;
			desiredVecRear.phi = phi;
			desiredVecFront.phi = saveFrontFlippperPossition*R2D;
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
			phi = snakeMode.snakeModeAngle (zAtStepZ);
			desiredVecRear.z = zAtStepZ;
			desiredVecRear.x = xAtStepZ;
			desiredVecRear.phi = phi;
			//setVelocityLimitation (false);
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
			snakeMode.snakeModeVec (distanceVecRear, desiredVecRear);
			break;
		}
		default:
		{

			break;
		}
	}
}


/**
 * Mode to chose driving forward or backward a step down
 */
void StepBehavior::downwardStepMode (double& frontDesiredAngle, double& rearDesiredAngle)
{
	std::cout << "---------downwardStepMode-------- " << std::endl;
	VecXZPhi desiredVecFront;
	VecXZPhi desiredVecRear;
	std::cout<<"robotDirection"<<robotDirection<<std::endl;

	switch (robotDirection)
	{

		// Robot moves backwards
		case -1:
		{
			//rear.vd=mid2_uss;
			downwardStepModeVec (rear, front, desiredVecRear, desiredVecFront);
			break;
		}
			// Robot moves forward
		case 1:
		{
			//front.vd=mid1_uss;
			downwardStepModeVec (front, rear, desiredVecFront, desiredVecRear);
			break;
		}
			// Snake mode
		default:
		{
			snakeMode.snakeModeVec (front, desiredVecFront);
			snakeMode.snakeModeVec (rear, desiredVecRear);
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

void StepBehavior::downwardStepModeVec (DistanceVec distanceVecFront, DistanceVec distanceVecRear, VecXZPhi &desiredVecFront, VecXZPhi &desiredVecRear)
{
	static double saveSnakeModeAngle;
	switch (stateStep)
	{
		case START:
		{

			int sizeVec = distanceVecFront.z.size ();
			std::cout<<"sizeVec"<<sizeVec<<std::endl;
			for (int row = 0; row < sizeVec - 1; row++)
			{
				std::cout<<"distanceVecFront.z.at (row)"<<	distanceVecFront.z.at (row)<<std::endl;

			}
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
			if (stepZ <= step_height_treshold)
			{
				stateStep = DETECTED;
			}
			break;
		}
		case DETECTED:
		{
			std::cout<<"DETECTED"<<std::endl;
			std::cout<<"distanceVecFront.vd"<<distanceVecFront.vd<<std::endl;
			//snakeMode.snakeModeVec (distanceVecFront, desiredVecFront);
			snakeMode.snakeModeVec (distanceVecRear, desiredVecRear);
			std::cout<<"distanceVecFront.vd "<<distanceVecFront.vd <<std::endl;
			if((distanceVecFront.vd >= step_height_treshold))
			{
				stateStep = MOVE_FORWARD_FLIPPER;
			}
			break;
		}
		case MOVE_FORWARD_FLIPPER:
		{
			std::cout<<"MOVE_FORWARD_FLIPPER"<<std::endl;
			snakeMode.snakeModeVec (distanceVecFront, desiredVecFront);
			saveSnakeModeAngle=desiredVecFront.phi;
			stateStep = WAIT_FOR_USS;
			break;
		}
		case WAIT_FOR_USS:
		{
			std::cout<<"WAIT_FOR_USS"<<std::endl;
			desiredVecFront.phi=saveSnakeModeAngle;
			if(distanceVecRear.vd >= step_height_treshold)
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
			/*distanceVecFront.z = front.z;
			distanceVecFront.x = front.x;
			distanceVecRear.z = rear.z;
			distanceVecRear.x = rear.x;*/
			snakeMode.snakeModeVec (distanceVecFront, desiredVecFront);
			snakeMode.snakeModeVecAbs(distanceVecRear, desiredVecRear);

			if ((mid2_uss < tempTreshholdDistanceRearUltrasound) and (distanceVecRear.vd < tempTreshholdDistanceRearUltrasound) and desiredVecRear.x >= tempTreshholdDistanceRearX)
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
			snakeMode.snakeModeVec (distanceVecFront, desiredVecFront);
			snakeMode.snakeModeVec (distanceVecRear, desiredVecRear);
			break;
		}
	}
}

/**
 * Detects the end of a stair in upward direction
 * @param vec values of the laser scanner
 * @return true if end is detected
 */

/**
 * Detects a upward step
 * @param vec values of the laser scanner
 * @return true if step is detected
 */
int StepBehavior::upwardStepDetection (DistanceVec vec)
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
 * Does a reset after a step was climbed
 */

/**
 * Detects a step in down direction
 * @param vec values of the laser scanner
 * @return true if it is a down step
 */
int StepBehavior::downwardStepDetection (DistanceVec vec)
{

	int sizeVec = vec.z.size ();
	std::cout<<"sizeVec"<<sizeVec<<std::endl;
	for (int row = 0; row < sizeVec - 1; row++)
	{
		int dz = std::fabs (vec.z.at (row) - vec.z.at (row + 1));
		std::cout<<"dz"<<dz<<std::endl;
		std::cout<<"dz"<<dz<<std::endl;

		if (dz >= step_height_treshold)
		{
			std::vector<double>::iterator minIt;
			double minVel;
			unsigned int minEl;
			minIt = std::min_element (vec.z.begin (), vec.z.end ());
			minEl = std::distance (vec.z.begin (), minIt);
			minVel = vec.z.at (minEl);
			if (minVel <= -step_height_treshold)
				std::cout<<"minVel"<<minVel<<std::endl;
				return 1;
		}
	}
	return 0;
}

void StepBehavior::stepReset ()
{
	stepBeta = 0.0;
	stepZ = 0.0;
	stateStep = START;
	currentState=searchStep;
}

void StepBehavior::reconfigureCallback (robot_navigation::StepBehaviorConfig &confg, uint32_t level)
{
	ROS_INFO ("************************************ Reconfigure callback ********************************************\n");

	step_mode_front_angle_min = confg.step_mode_front_angle_min;
	flipper_front_angle_step_climb = confg.flipper_front_angle_step_climb;
	body_angle_treshold = confg.body_angle_treshold;
	bodyAngleThreshold=confg.bodyAngleThreshold;
	step_offset = confg.step_offset;
	step_angle_treshold = confg.step_angle_treshold;
	step_rear_height_treshold = confg.step_rear_height_treshold;
	step_height_treshold = confg.step_height_treshold;
	step_distance_treshold_min = confg.step_distance_treshold_min;
	step_distance_treshold_max = confg.step_distance_treshold_max;
}


