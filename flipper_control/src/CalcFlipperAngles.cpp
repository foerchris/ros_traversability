/*
 * CalcFlipperAngles.cpp
 *
 *  Created on: 16.01.2019
 *      Author: chfo
 */
#include "CalcFlipperAngles.h"


CalcFlipperAngles::CalcFlipperAngles()
{
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
	FlipperTrackLength = 2*(xLength + R) + trackLength;
	TracksBaseLinkDist = 0.275;
}


CalcFlipperAngles::~CalcFlipperAngles()
{

}

std::vector<geometry_msgs::Pose> CalcFlipperAngles::isTrackInRange(const std::vector<geometry_msgs::Pose>& poses)
{
	std::vector<geometry_msgs::Pose> validPose;
	for(std::size_t i = 0; i<poses.size(); i++)
	{
		if(-FlipperTrackLength/2>poses[i].position.x-velocitiy_robot*delta_t && FlipperTrackLength/2<poses[i].position.x-velocitiy_robot*delta_t)
		{
			validPose.push_back(poses[i]);
		}
	}

}

geometry_msgs::Pose CalcFlipperAngles::clcDesiredPose(const std::vector<geometry_msgs::Pose>& poses)
{
	geometry_msgs::Pose desiredPose;
	geometry_msgs::Pose meanPose;
	std::vector<double> crossMeanPose;
	meanPose = clcMean(poses);
	crossMeanPose = clcCrossMean(poses);

	double meanXX = crossMeanPose[0];
	double meanXY = crossMeanPose[1];
	double meanYY = crossMeanPose[2];
	double meanYZ = crossMeanPose[3];
	double meanZX = crossMeanPose[4];

	double alpha_xx = meanXX - meanPose.position.x*meanPose.position.x;
	double alpha_xy = meanXY - meanPose.position.x*meanPose.position.y;
	double alpha_yy = meanYY - meanPose.position.y*meanPose.position.y;
	double alpha_yz = meanYZ - meanPose.position.y*meanPose.position.z;
	double alpha_zx = meanZX - meanPose.position.z*meanPose.position.x;

	double a = (alpha_zx*alpha_yy - alpha_xy*alpha_yz)/(alpha_xx*alpha_yy - alpha_xy*alpha_xy);
	double b = (alpha_yz*alpha_xx - alpha_xy*alpha_zx)/(alpha_xx*alpha_yy - alpha_xy*alpha_xy);
	double c = meanPose.position.z - meanPose.position.x*a - meanPose.position.y*b;

	//double aplhaXY = meanPose.position.x *  meanPose.position.y

	//double a = ()
	return desiredPose;
}

geometry_msgs::Pose CalcFlipperAngles::clcMean(const std::vector<geometry_msgs::Pose>& poses)
{
	geometry_msgs::Pose mean;
	mean.position.x = 0;
	mean.position.y = 0;
	mean.position.z = 0;


	int N = poses.size();
	for(int i=0; i<N;i++)
	{
		mean.position.x = mean.position.x + poses[i].position.x;
		mean.position.y = mean.position.y + poses[i].position.y;
		mean.position.z = mean.position.z + poses[i].position.z;
	}
	mean.position.x = mean.position.x/N;
	mean.position.y = mean.position.y/N;
	mean.position.z = mean.position.z/N;

	return mean;
}

std::vector<double> CalcFlipperAngles::clcCrossMean(const std::vector<geometry_msgs::Pose>& poses)
{
	std::vector<double> crossMean;

	double meanXX;
	double meanXY;
	double meanYY;
	double meanYZ;
	double meanZX;

	int N = poses.size();
	for(int i=0; i<N;i++)
	{
		meanXX = meanXX + poses[i].position.x*poses[i].position.x;
		meanXY = meanXY + poses[i].position.x*poses[i].position.y;
		meanYY = meanYY + poses[i].position.y*poses[i].position.y;
		meanYZ = meanYZ + poses[i].position.y*poses[i].position.z;
		meanZX = meanZX + poses[i].position.z*poses[i].position.x;

	}
	meanXX = meanXX/N;
	meanXY = meanXY/N;
	meanYY = meanYY/N;
    meanYZ = meanYZ/N;
	meanZX = meanZX/N;

	crossMean.push_back(meanXX);
	crossMean.push_back(meanXY);
	crossMean.push_back(meanYY);
	crossMean.push_back(meanYZ);
	crossMean.push_back(meanZX);

	return crossMean;
}

