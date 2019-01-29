/*
 * CalcFlipperAngles.cpp
 *
 *  Created on: 16.01.2019
 *      Author: chfo
 */
#include "CalcFlipperAngles.h"
#include "math.h"


CalcFlipperAngles::CalcFlipperAngles()
{

}


CalcFlipperAngles::~CalcFlipperAngles()
{

}

flipperContactPointsAngles CalcFlipperAngles::clcContactAngles(const std::vector<geometry_msgs::Pose>& values)
{
	flipperContactPointsAngles robotFlipperAngles;

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
		d= sqrt(pow(x, 2)+ pow(z, 2));

		if(d<= dThreshold)
		{
			phi1 = atan(z/x);
			phi2 = asin(R/sqrt(pow(x, 2) + pow(z,2)));
			phiContact = phi1 + phi2;

			//********************************************************** nochmal nachrechnen **********************************************************
			flipperAngle = phi1 - asin((R-r)/L);
		}
		else
		{
			phi1 = asin((pow(d,2) + pow(L,2) - pow(R,2))/ (2*L*d)) - atan(x/z);
			phi2 = asin((R-r)/L);
			phiContact = phi1 + phi2;

			//********************************************************** nochmal nachrechnen **********************************************************
			flipperAngle = phi1;
		}
		robotFlipperAngles.pose.push_back(pose);
		robotFlipperAngles.phi1.push_back(phi1);
		robotFlipperAngles.phi2.push_back(phi2);
		robotFlipperAngles.phiContact.push_back(phiContact);
		robotFlipperAngles.flipperAngle.push_back(flipperAngle);
	}

	return robotFlipperAngles;
}



