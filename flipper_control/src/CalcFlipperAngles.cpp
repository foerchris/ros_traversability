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
	dThreshold=0;
	R=0;
	L=0;
	r=0;

}


CalcFlipperAngles::~CalcFlipperAngles()
{

}
void CalcFlipperAngles::setParameter(double p1, double p2, double p3, double p4)
{
	dThreshold = p1;
	R = p2;
	r = p3;
	L = p4;

}

double CalcFlipperAngles::clcContactAngles(const std::vector<geometry_msgs::Pose>& values)
{
	flipperContactPointsAngles robotFlipperAngles;
	static flipperContactPointsAngles lastrobotFlipperAngles;

	double z = 0;
	double x = 0;
	double d = 0;
	for(auto pose:values)
	{
		//ROS_INFO("pose: x = %7.3lf, y = %7.3lf, z = %7.3lf", pose.position.x, pose.position.y, pose.position.z);

	}
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

			/*ROS_INFO("x = %7.3lf", x);
			ROS_INFO("z = %7.3lf", z);

			ROS_INFO("d = %7.3lf", d);
			ROS_INFO("dThreshold = %7.3lf", dThreshold);
*/

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
			robotFlipperAngles.pose.push_back(pose);
			robotFlipperAngles.phi1.push_back(phi1);
			robotFlipperAngles.phi2.push_back(phi2);
			robotFlipperAngles.phiContact.push_back(phiContact);
			robotFlipperAngles.flipperAngle.push_back(flipperAngle);
			/*ROS_INFO("phi1 = %7.3lf", phi1/M_PI*180);
			ROS_INFO("phi2 = %7.3lf", phi2/M_PI*180);
			ROS_INFO("phiContact = %7.3lf", phiContact/M_PI*180);
			ROS_INFO("flipperAngle = %7.3lf", flipperAngle/M_PI*180);
*/
		}
	}
	if(robotFlipperAngles.phiContact.size() == 0)
	{
		robotFlipperAngles = lastrobotFlipperAngles;
	}
	else
	{
		lastrobotFlipperAngles = robotFlipperAngles;
	}


	//ROS_INFO("maxFlipperAngle(robotFlipperAngles) = %7.3lf", maxFlipperAngle(robotFlipperAngles)/M_PI*180);

	return maxFlipperAngle(robotFlipperAngles);
}


double CalcFlipperAngles::maxFlipperAngle(const flipperContactPointsAngles& flipperAngles)
{
	double maxFlipperAngle = 0;

	auto iter = std::max_element(flipperAngles.phiContact.begin(), flipperAngles.phiContact.end());

	maxFlipperAngle =*iter;

	return maxFlipperAngle;
}
