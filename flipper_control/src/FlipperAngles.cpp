/*
 * FlipperAngles.cpp
 *
 *  Created on: 16.01.2019
 *      Author: chfo
 */
#include <flipper_control/FlipperAngles.h>

#include "math.h"


FlipperAngles::FlipperAngles()
{
	dThreshold=0;
	R=0;
	L=0;
	r=0;

}


FlipperAngles::~FlipperAngles()
{

}
void FlipperAngles::setParameter(double p1, double p2, double p3, double p4)
{
	dThreshold = p1;
	R = p2;
	r = p3;
	L = p4;

}

MaxFlipperContactPointsAngles FlipperAngles::clcContactAngles(const std::vector<geometry_msgs::Pose>& values)
{
	FlipperContactPointsAngles flipperContactPointsAngles;
	static FlipperContactPointsAngles lastFlipperContactPointsAngles;

	double z = 0;
	double x = 0;
	double d = 0;


	for(auto pose:values)
	{
		double phi1 = 0;
		double phi2 = 0;
		double phiContact = 0;
		double flipperAngle = 0 ;

		z = pose.position.z;

		x = pose.position.x;
		if(x >= 0)
		{
			d= sqrt(pow(x, 2)+ pow(z, 2));
			//ROS_INFO("pose: x = %3.6lf, y = %3.6lf, z = %3.6lf", x, pose.position.y, z);

			//if(d<= dThreshold)
			//{
				phi1 = atan(z/x);
				phi2 = asin(R/sqrt(pow(x, 2) + pow(z,2)));
				phiContact = phi1 + phi2;

				//********************************************************** nochmal nachrechnen **********************************************************
				//flipperAngle = phi1 - asin((R-r)/L);
				flipperAngle = phiContact;

			/*}
			else
			{
				phi1 = asin((pow(d,2) + pow(L,2) - pow(R,2))/ (2*L*d)) - atan(x/z);
				phi2 = asin((R-r)/L);
				phiContact = phi1 + phi2;

				//********************************************************** nochmal nachrechnen **********************************************************
				//flipperAngle = phi1;
				flipperAngle = phiContact;
			}*/
				//ROS_INFO("phiContact = %3.6lf", phiContact/M_PI*180);

			flipperContactPointsAngles.pose.push_back(pose);
			flipperContactPointsAngles.phi1.push_back(phi1);
			flipperContactPointsAngles.phi2.push_back(phi2);
			flipperContactPointsAngles.phiContact.push_back(phiContact);
			flipperContactPointsAngles.flipperAngle.push_back(flipperAngle);

		}
	}
	if(flipperContactPointsAngles.phiContact.size() == 0)
	{
		flipperContactPointsAngles = lastFlipperContactPointsAngles;
	}
	else
	{
		lastFlipperContactPointsAngles = flipperContactPointsAngles;
	}


	//ROS_INFO("maxFlipperAngle(flipperContactPointsAngles) = %7.3lf", maxFlipperAngle(flipperContactPointsAngles)/M_PI*180);

	return maxFlipperAngle(flipperContactPointsAngles);
}


MaxFlipperContactPointsAngles FlipperAngles::maxFlipperAngle(const FlipperContactPointsAngles& flipperContactPointsAngles)
{
	MaxFlipperContactPointsAngles maxFlipperContactPointsAngles;

	auto iter = std::max_element(flipperContactPointsAngles.phiContact.begin(), flipperContactPointsAngles.phiContact.end());

	maxFlipperContactPointsAngles.maxFlipperAngle =*iter;
	int idx = iter - flipperContactPointsAngles.phiContact.begin();
	maxFlipperContactPointsAngles.pose = flipperContactPointsAngles.pose[idx];
	if(maxFlipperContactPointsAngles.maxFlipperAngle != flipperContactPointsAngles.phiContact[idx])
	{
		ROS_ERROR("This should not happen");
	}
	return maxFlipperContactPointsAngles;
}

