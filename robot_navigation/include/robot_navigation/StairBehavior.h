/*
 * StairBehavior.h
 *
 *  Created on: 13.05.2018
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_STAIRBEHAVIOR_H_
#define ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_STAIRBEHAVIOR_H_

#include "SnakeMode.h"
#include <robot_navigation/StairBehaviorConfig.h>

class StairBehavior
{
	public:
		StairBehavior ();
		virtual ~StairBehavior ();
		void SetBodyAngle (double pitch, double roll, double angle);
		void DistVect (DistanceVec frontDist, DistanceVec rearDist,DistanceVec frontRotBodyAngle, DistanceVec rearRotBodyAngle);
		void stairDownMode (double stairAngle, double stepHeight,double &frontDesiredAngle,double &rearDesiredAngle);
		void stairUpMode (double stairAngle, double stepHeight,double &frontDesiredAngle,double &rearDesiredAngle);
		void SetUss (double front, double mid1, double mid2, double rear);
		void SetStairBehaviorParams (double flipperAngleMax, double flipperAngleMin);
		void MoveRobot (bool& move);


	private:
		bool upwardEndOfStairDetection (DistanceVec vec);
		void reconfigureCallback (robot_navigation::StairBehaviorConfig &confg, uint32_t level);
		enum stairClimbEnum
		{
			start, ptichUp,startClimbing, normalClimbing, finalClimbing, pitchDown, reset
		};
		stairClimbEnum stairUpStat;
		stairClimbEnum stairDownStat;

		bool moveRobot;
		double bodyAngleRoll;
		double bodyAnglePitch;
		double bodyAngle;
		double front_uss;
		double mid1_uss;
		double mid2_uss;
		double rear_uss;

		DistanceVec front;
		DistanceVec rear;
		DistanceVec front_rotBodyAngle;
		DistanceVec rear_rotBodyAngle;

		ros::Timer timer;
		double currentTime;
		double previousTime;
		SnakeMode snakeMode;
};
#endif /* ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_STAIRBEHAVIOR_H_ */
