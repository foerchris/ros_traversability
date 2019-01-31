/*
 * FitPlane.cpp
 *
 *  Created on: 17.01.2019
 *      Author: chfo
 */

#include "FitPlane.h"
#include "math.h"
#include "tf/transform_datatypes.h"
FitPlane::FitPlane()
{

}


FitPlane::~FitPlane()
{

}
FittedPlane FitPlane::fitPlane(const std::vector<geometry_msgs::Pose>& poses)
{
	geometry_msgs::Pose meanPose;
	std::vector<double> crossMeanPose;
	FittedPlane fittedPlane;

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

	fittedPlane.a = (alpha_zx*alpha_yy - alpha_xy*alpha_yz)/(alpha_xx*alpha_yy - alpha_xy*alpha_xy);
	fittedPlane.b = (alpha_yz*alpha_xx - alpha_xy*alpha_zx)/(alpha_xx*alpha_yy - alpha_xy*alpha_xy);

	fittedPlane.c = meanPose.position.z - meanPose.position.x*fittedPlane.a - meanPose.position.y*fittedPlane.b;
	return fittedPlane;
}

std::vector<geometry_msgs::Pose> FitPlane::samplePlane(const FittedPlane& fittedPlane,const double& xLength,const double& yLength,const double& resulution)
{
	std::vector<geometry_msgs::Pose> planePoints;
	geometry_msgs::Pose point;
	double x = -xLength/2;
	double y = -yLength/2;
	double deltax = xLength*resulution;
	double deltay = yLength*resulution;


	while(x <= xLength/2)
	{

		while(y <= yLength/2)
		{

			point.position.x = x;
			point.position.y = y;
			point.position.z = fittedPlane.a * x + fittedPlane.b *y + fittedPlane.c;
			planePoints.push_back(point);
			y += deltay;
		}
		x += deltax;
		y = -yLength/2;
	}
	return planePoints;
}


std::vector<geometry_msgs::Pose> FitPlane::sampleLine(const double& angle ,const double& xLength,const double& resulution)
{
	std::vector<geometry_msgs::Pose> planePoints;
	geometry_msgs::Pose point;
	double x = 0;
	double deltax = xLength*resulution;

	double a= atan(angle-M_PI/12);

	while(x <= xLength)
	{

			point.position.x = x;
			point.position.y = 0;
			point.position.z = a * x;
			planePoints.push_back(point);
		x += deltax;
	}
	return planePoints;
}

tf2::Quaternion FitPlane::getRotations(FittedPlane fittedPlane)
{
	double theta_rot = acos(1/sqrt(fittedPlane.a*fittedPlane.a+fittedPlane.b*fittedPlane.b+1));

	double x = fittedPlane.b*sin(theta_rot/2);
	double y = -fittedPlane.a*sin(theta_rot/2);
	double z = 0;
	double w = cos(theta_rot/2);
	tf2::Quaternion q;

	if(x!=x || y!=y || z!=z || w!=w)
	{
		q.setRPY(0, 0, 0);

	}
	else
	{
		/*q.setX(w);
		q.setY(x);
		q.setZ(y);
		q.setW(z);*/
		q.setX(x);
		q.setY(y);
		q.setZ(z);
		q.setW(w);
	}
	return q;
}

geometry_msgs::Pose FitPlane::clcMean(const std::vector<geometry_msgs::Pose>& poses)
{
	geometry_msgs::Pose mean;
	mean.position.x = 0;
	mean.position.y = 0;
	mean.position.z = 0;


	int N = 0;
	for(auto pose : poses)
	{
			mean.position.x = mean.position.x + pose.position.x;
			mean.position.y = mean.position.y + pose.position.y;
			mean.position.z = mean.position.z + pose.position.z;
			 N +=1;
	}

	mean.position.x = mean.position.x/N;
	mean.position.y = mean.position.y/N;
	mean.position.z = mean.position.z/N;
	return mean;
}

std::vector<double> FitPlane::clcCrossMean(const std::vector<geometry_msgs::Pose>& poses)
{
	std::vector<double> crossMean;

	double meanXX = 0;
	double meanXY = 0;
	double meanYY = 0;
	double meanYZ = 0;
	double meanZX = 0;

	int N = 0;
	for(auto pose : poses)
	{
		meanXX = meanXX + pose.position.x*pose.position.x;
		meanXY = meanXY + pose.position.x*pose.position.y;
		meanYY = meanYY + pose.position.y*pose.position.y;
		meanYZ = meanYZ + pose.position.y*pose.position.z;
		meanZX = meanZX + pose.position.z*pose.position.x;
		N += 1;
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




