/*
 * StepBehavior.h
 *
 *  Created on: 21.05.2018
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_STEPBEHAVIOR_H_
#define ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_STEPBEHAVIOR_H_

#include "SnakeMode.h"
#include <robot_navigation/StepBehaviorConfig.h>

class StepBehavior
{
	public:
	StepBehavior ();
		virtual ~StepBehavior ();
		void SetBodyAngle (double pitch, double roll, double angle);
		void DistVect (DistanceVec frontDist, DistanceVec rearDist, double currentFrontAngle, double currentRearAngle, int direction);
		void SetUss (double front, double mid1, double mid2, double rear);
		void SetStepBehaviorParams (double flipperAngleMax, double flipperAngleMin);
		void upwardStepMode (double& frontDesiredAngle, double& rearDesiredAngle);
		void downwardStepMode (double& frontDesiredAngle, double& rearDesiredAngle);
		void stepReset ();


	private:
		void reconfigureCallback (robot_navigation::StepBehaviorConfig &confg, uint32_t level);
		void upwardStepModeVec (DistanceVec distanceVecFront, DistanceVec distanceVecRear, VecXZPhi &desiredVecFront, VecXZPhi &desiredVecRear, double currentRearFlippperPossition, double currentFrontFlippperPossition);
		void downwardStepModeVec (DistanceVec distanceVecFront, DistanceVec distanceVecRear, VecXZPhi &desiredVecFront, VecXZPhi &desiredVecRear);
		int upwardStepDetection (DistanceVec vec);
		int downwardStepDetection (DistanceVec vec);

		enum StateDownwardStep
		{
			RESET, START, SEARCH, DETECTED, MOVE_FORWARD_FLIPPER,WAIT_FOR_USS, MOVE_BACKWARD_FLIPPER, FINISHED
		};

		enum upStepState
		{
			searchStep, stepInitalisieren, approachStep,moveRearFlipper, waitForRearUSS, setRearFlipperHorizontal, stepOvercome
		};

		upStepState currentState;

		double bodyAngleRoll;
		double bodyAnglePitch;
		double bodyAngle;
		double front_uss;
		double mid1_uss;
		double mid2_uss;
		double rear_uss;

		double L;		// Wheel distance between the robot axes
		double R;		// Radius of the wheel of the robot body
		double l;		// Wheel distance between the robot wheel axis and the wheel axis on the end of the flipper
		double r;		// Radius of the wheel on the end of the flipper
		double d;		// Distance between body wheel axis and "downside of flipper wheel"

		DistanceVec front;
		DistanceVec rear;
		double currentFlipperAngleFront;
		double currentFlipperAngleRear;
		int robotDirection;
		int stateStep;
		double stepBeta;
		double stepZ;
		SnakeMode snakeMode;

		double body_angle_treshold;
		double flipper_front_angle_step_climb;
		double bodyAngleThreshold;
		double step_mode_front_angle_min;
		double step_height_treshold;
		double step_distance_treshold_min;
		double step_distance_treshold_max;
		double step_rear_height_treshold;
		double step_angle_treshold;
		double step_offset;

};



#endif /* ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_STEPBEHAVIOR_H_ */
