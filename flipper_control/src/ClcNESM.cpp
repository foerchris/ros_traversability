/*
 * ClcNESM.cpp
 *
 *  Created on: 01.02.2019
 *      Author: chfo
 */

#include "include/ClcNESM.h"

ClcNESM::ClcNESM()
{
	NESM_thresshold = 0.5; //// SET to the right value

}

ClcNESM::~ClcNESM()
{

}


double ClcNESM::clcNESMStabilityMeasure(const geometry_msgs::Pose& contactPointFront, const geometry_msgs::Pose& contactPointRear, const geometry_msgs::Pose& centerOfGravity, const bool& rotatePitch,const int& rotationDirection)
{
	cv::Vec3d g1(contactPointFront.position.x, contactPointFront.position.y, contactPointFront.position.z);
	cv::Vec3d g2(contactPointRear.position.x, contactPointRear.position.y, contactPointRear.position.z);
	cv::Vec3d c(centerOfGravity.position.x, centerOfGravity.position.y, centerOfGravity.position.z);
	g1_public = g1;
	g2_public = g2;
	c_public = c;
	cv::Vec3d offset = g1;
	//g1 = g1 - offset;
	//g2 = g2 - offset;
	//c = c - offset;

	tf2::Quaternion quat;
	if(rotatePitch)
	{
		quat.setRPY(0,rotationDirection*M_PI/2,0);
	}
	else
	{
		quat.setRPY(rotationDirection*M_PI/2,0,0);
	}

	cv::Vec3d g1_prime = clcQuaternion(g1,quat) + offset;

	cv::Vec3d g2_prime = clcQuaternion(g2,quat) + offset;

	cv::Vec3d c_prime = clcQuaternion(c,quat) + offset;

	g1_prime_public = g1_prime;
	g2_prime_public = g2_prime;
	c_prime_public = c_prime;

	cv::Vec3d ug = g2_prime - g1_prime;
	cv::Vec3d uc = c_prime - g1_prime;

	cv::Vec3d added = magnitude(ug.dot(uc))/magnitude(ug)*ug/magnitude(ug);
	ROS_INFO(" magnitude(ug.dot(uc))/magnitude(ug)*ug/magnitude(ug): x = %4.4lf, y = %4.4lf, z = %4.4lf",added[0],added[1],added[2]);

	cv::Vec3d p_foot = g1_prime + magnitude(ug.dot(uc))/magnitude(ug)*ug/magnitude(ug);

	ug_public = ug;
	uc_public = uc;
	p_foot_public = p_foot;

	cv::Vec3d unitVecZ(0,0,1);
	cv::Vec3d u_highest = unitVecZ - magnitude(unitVecZ.dot(ug))/magnitude(ug) * ug/magnitude(ug);

	u_highest_public = u_highest;

	cv::Vec3d p_highest = p_foot + magnitude(p_foot-c_prime)*(u_highest)/(magnitude(u_highest));

	p_highest_public = p_highest;
	double S_NE = p_highest[2] - c_prime[2];
	return S_NE;
}

double ClcNESM::magnitude(const cv::Vec3d& v)
{
	return sqrt(pow(v[0],2)+ pow(v[1],2)+ pow(v[2],2));
}



cv::Vec3d ClcNESM::clcQuaternion(const cv::Vec3d& v,const tf2::Quaternion& q)
{
    // Extract the vector part of the quaternion
	cv::Vec3d u(q.getX(), q.getY(), q.getZ());
    // Extract the scalar part of the quaternion
    float s = q.getW();

    // Do the math
    cv::Vec3d   vprime = 2.0f * u.dot(v) * u + (s*s - u.dot(u)) * v + 2.0f * s * u.cross(v);

	return vprime;
}
