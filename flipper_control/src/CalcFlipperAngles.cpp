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
	tfListener = std::unique_ptr<tf::TransformListener> (new tf::TransformListener);

	// Roboter paramter
	R = 0.082;
	r = 0.0375;
	//L = 0.22;
	L = 0.2325;
	//d = std::hypot (r, L);
	theta = atan (r / L);
	dThreshold = sqrt(pow(R,2) + pow(L,2) - pow(R-r,2));
	//xLength  = 0.1;
	yLength  = 0.1;
	//yLength  = 0.2575;
	//	yLength = L+r;
	xLength = L+r;
	trackLength = 0.5;
	FlipperTrackLength = 2*(xLength - R) + trackLength;
	TracksBaseLinkDist = 0.275;
}


CalcFlipperAngles::~CalcFlipperAngles()
{

}

std::vector<geometry_msgs::Pose> CalcFlipperAngles::clcNewPoses(const std::vector<geometry_msgs::Pose>& poses, tf2::Quaternion q)
{
	tf2::Quaternion q_prime = q.inverse();
	tf2::Quaternion p;
	p.setX(0);
	tf2::Quaternion p_prime;

	std::vector<geometry_msgs::Pose> newPoses;
	geometry_msgs::Pose pose_prime;
	for(auto pose : poses)
	{
		//ROS_INFO (" pose:\t x = [%7.3lf], y = [%7.3lf], z = [%7.3lf]", pose.position.x, pose.position.y, pose.position.z);
		p.setY(pose.position.x);
		p.setZ(pose.position.y);
		p.setW(pose.position.z);

		p_prime=q*p*q_prime;
		pose_prime.position.x = p_prime.getY();
		pose_prime.position.y = p_prime.getZ();
		pose_prime.position.z = p_prime.getW();
		newPoses.push_back(pose_prime);
		//ROS_INFO (" pose_prime:\t x = [%7.3lf], y = [%7.3lf], z = [%7.3lf]", pose_prime.position.x, pose_prime.position.y, pose_prime.position.z);

	}

	double maxZ = 0;
	for(auto pose : newPoses)
	{
		if(pose.position.z > maxZ)
		{
			maxZ = pose.position.z;
		}
	}
	//ROS_INFO (" newPoses[i]:\t x = [%7.3lf], y = [%7.3lf], z = [%7.3lf]", newPoses[i].position.x, newPoses[i].position.y, newPoses[i].position.z);

	for(std::size_t i=0; i < newPoses.size(); i++)
	{
		newPoses[i].position.z = newPoses[i].position.z - maxZ;
		//ROS_INFO (" newPoses[i]:\t x = [%7.3lf], y = [%7.3lf], z = [%7.3lf]", newPoses[i].position.x, newPoses[i].position.y, newPoses[i].position.z);

	}
	return newPoses;
}

double CalcFlipperAngles::maxFlipperAngle(const flipperContactPointsAngles& flipperAngles)
{
	double maxFlipperAngle = 0;

	auto iter = std::max_element(flipperAngles.phiContact.begin(), flipperAngles.phiContact.end());

	maxFlipperAngle =*iter;

	return maxFlipperAngle;
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


