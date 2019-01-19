/*
 * FitPlane.cpp
 *
 *  Created on: 17.01.2019
 *      Author: chfo
 */

#include "FitPlane.h"
#include "math.h"

FitPlane::FitPlane()
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


FitPlane::~FitPlane()
{

}


tf2::Quaternion FitPlane::fitPlane(const std::vector<geometry_msgs::Pose>& poses)
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
	double theta_rot = acos(1/sqrt(a*a+b*b+1));

	ROS_INFO (" a:\t  [%7.3lf]\n", a);
	ROS_INFO (" b:\t  [%7.3lf]\n", b);
	ROS_INFO (" c:\t  [%7.3lf]\n", c);


	tf2::Quaternion quat;

	quat.setX(cos(theta_rot/2));
	quat.setY(b*sin(theta_rot/2));
	quat.setZ(-a*sin(theta_rot/2));
	quat.setW(0);
	return quat;
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

std::vector<geometry_msgs::Pose> FitPlane::isTrackInRange(const std::vector<geometry_msgs::Pose>& poses, const double& velocitiy_robot, const double& delta_t)
{
	std::vector<geometry_msgs::Pose> robotGroundPose;


	for(auto pose:poses)
	{
		/*ROS_INFO (" -FlipperTrackLength/2:\t  [%7.3lf]", -FlipperTrackLength/2);
		ROS_INFO (" <pose.position.x-velocitiy_robot*delta_t:\t  [%7.3lf]\n", pose.position.x-velocitiy_robot*delta_t);
		ROS_INFO (" FlipperTrackLength/2:\t  [%7.3lf]", FlipperTrackLength/2);
		ROS_INFO (" >pose.position.x-velocitiy_robot*delta_t:\t  [%7.3lf]\n", pose.position.x-velocitiy_robot*delta_t);
*/

		if(-FlipperTrackLength/2<pose.position.x-velocitiy_robot*delta_t && FlipperTrackLength/2>pose.position.x-velocitiy_robot*delta_t)
		{
			robotGroundPose.push_back(pose);
		}

	}

	return robotGroundPose;
}




