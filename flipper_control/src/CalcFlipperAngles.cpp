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

/*
void rotate_vector_by_quaternion(const tf2::Vector3& v, const tf2::Quaternion& q, tf2::Vector3& vprime)
{
    // Extract the vector part of the quaternion
	tf2::Vector3 u(q.x, q.y, q.z);

    // Extract the scalar part of the quaternion
    float s = q.w;

    // Do the math
    vprime = 2.0f * dot(u, v) * u
          + (s*s - dot(u, u)) * v
          + 2.0f * s * cross(u, v);
}
*/

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
		p.setY(pose.position.x);
		p.setZ(pose.position.y);
		p.setW(pose.position.z);

		p_prime=q*p*q_prime;
		pose_prime.position.x = p_prime.getY();
		pose_prime.position.y = p_prime.getZ();
		pose_prime.position.z = p_prime.getW();
		newPoses.push_back(pose_prime);
	}

	double maxZ = 0;
	for(auto pose : newPoses)
	{
		if(pose.position.z > maxZ)
		{
			pose.position.z = maxZ;
		}
	}

	for(std::size_t i=0; i < newPoses.size(); i++)
	{
		newPoses[i].position.z = newPoses[i].position.z - maxZ;
	}

	return newPoses;
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
	return validPose;

}


