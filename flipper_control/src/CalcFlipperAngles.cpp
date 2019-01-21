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

/*

flipperContactPointsAngles CalcFlipperAngles::clcContactAngles(const std::vector<geometry_msgs::Pose>& values, std::string flipperFrame)
{
	flipperContactPointsAngles robotFlipperAngles;

	geometry_msgs::Pose flipperPose;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = 0;
	tf2::Quaternion quat;
	quat.setRPY(0,0, 0);
	flipperPose.orientation.x = quat.x();
	flipperPose.orientation.y = quat.y();
	flipperPose.orientation.z = quat.z();
	flipperPose.orientation.w = quat.w();

	double z = 0;
	double x = 0;
	double d = 0;
	for(std::size_t i = 0; i <values.size(); i++)
	{
		geometry_msgs::Pose point;

		double phi1;
		double phi2;
		double phiContact;
		double flipperAngle;

		point = values[i];


		z = values[i].position.z;
		x = values[i].position.x;

		flipperPose.position.z = z;
		flipperPose = mapToFlipperTransform(flipperPose, tf_prefix + flipperFrame);
		z = flipperPose.position.z;

		point.position.z=z;

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
		robotFlipperAngles.pose.push_back(point);
		robotFlipperAngles.phi1.push_back(phi1);
		robotFlipperAngles.phi2.push_back(phi2);
		robotFlipperAngles.phiContact.push_back(phiContact);
		robotFlipperAngles.flipperAngle.push_back(flipperAngle);

	}

	return robotFlipperAngles;
}

void CalcFlipperAngles::clcFlipperAngles(const std::vector<minMaxFlipperVel>& minMaxVel, flipperAngles& robotFlipperAngles)
{
	geometry_msgs::Pose flipperPose;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = 0;
	tf2::Quaternion quat;
	quat.setRPY(0,0, 0);
	flipperPose.orientation.x = quat.x();
	flipperPose.orientation.y = quat.y();
	flipperPose.orientation.z = quat.z();
	flipperPose.orientation.w = quat.w();


	double zFrontLeftMax = minMaxVel[0].max*0.0043;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = zFrontLeftMax;
	flipperPose = mapToFlipperTransform(flipperPose, tf_prefix + "/static_flipper_front_left");
	zFrontLeftMax = flipperPose.position.z;

	double zFrontRightMax = minMaxVel[1].max*0.0043;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = zFrontRightMax;
	flipperPose = mapToFlipperTransform(flipperPose, tf_prefix + "/static_flipper_front_right");
	zFrontRightMax = flipperPose.position.z;

	double zRearLeftMax = minMaxVel[2].max*0.0043;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = zRearLeftMax;
	flipperPose = tfTransform(flipperPose, tf_prefix + "/static_flipper_rear_left");
	zRearLeftMax = flipperPose.position.z;

	double zRearRightMax = minMaxVel[3].max*0.0043;
	flipperPose.position.x = 0;
	flipperPose.position.y = 0;
	flipperPose.position.z = zRearRightMax;
	flipperPose = tfTransform(flipperPose, tf_prefix + "/static_flipper_rear_right");
	zRearRightMax = flipperPose.position.z;

	double xFront = 0;
	double zFront = 0;
	double xRear  = 0;
	double zRear  = 0;

	if(zFrontRightMax > zFrontLeftMax)
	{
		xFront = yLength - minMaxVel[1].max_loc.y*resultion;
		zFront = zFrontRightMax;
	}
	else
	{
		xFront = yLength - minMaxVel[0].max_loc.y*resultion;
		zFront = zFrontLeftMax;
	}

	if(zRearRightMax > zRearLeftMax)
	{
		xRear = yLength - minMaxVel[3].max_loc.y*resultion;
		zRear = zRearRightMax;

	}
	else
	{
		xRear = yLength - minMaxVel[2].max_loc.y*resultion;
		zRear = zRearLeftMax;

	}
	ROS_INFO("xFront = %lf, zFront = %lf\n", xFront, zFront);
	ROS_INFO("xRear = %lf, zRear = %lf\n", xRear, zRear);

	double dFront;
	double dRear;

	dFront = sqrt( pow(zFront, 2) + pow(xFront, 2));
	dRear = sqrt( pow(zRear, 2) + pow(xRear, 2));
	//ROS_INFO("dFront = %lf, dRear = %lf\n", dFront, dRear);

	// to do **********************************************************************************
	/// nochmal genau die einzelnden zustände nachdenken gerade werden negative hindernisse nicht perfekt behandelt
	if(zFront!=0)
	{
		if(xFront!=0)
		{
			double phi1 = 0;
			double phi2 = 0;
			double phiContact = 0;
			if(dFront <= dThreshold)
			{
				phi1 = atan(zFront/xFront);
				phi2 = asin(R/sqrt(pow(xFront, 2) + pow(zFront,2)));
				phiContact = phi1 + phi2;
				robotFlipperAngles.flipperAngleFront = phi1;
			}
			else
			{
				phi1 = asin((pow(dFront,2) + pow(L,2) - pow(R,2))/ (2*L*dFront)) - atan(xFront/zFront);
				phi2 = asin((R-r)/L);
				phiContact = phi1 + phi2;

				robotFlipperAngles.flipperAngleFront = phi1;
			}
			ROS_INFO("phi1 = %lf, phi2 = %lf, phiContact = %lf\n", phi1, phi2, phiContact);
			ROS_INFO("robotFlipperAngles.flipperAngleFront = %lf\n", robotFlipperAngles.flipperAngleFront);

		}
		else
		{
			robotFlipperAngles.flipperAngleFront = 85;
		}
	}
	else
	{
		robotFlipperAngles.flipperAngleFront = 0;
	}

	if(zRear!=0)
	{
		if(xRear!=0)
		{
			double phi1 = 0;
			double phi2 = 0;
			double phiContact = 0;
			if(dRear <= dThreshold)
			{
				phi1 = atan(zRear/xRear);
				phi2 = asin(R/sqrt(pow(xRear, 2) + pow(zRear,2)));
				phiContact = phi1 + phi2;

				robotFlipperAngles.flipperAngleRear = phi1;
			}
			else
			{
				phi1 = asin((pow(dRear,2) + pow(L,2) - pow(R,2))/ (2*L*dRear)) - atan(xRear/zRear);
				phi2 = asin((R-r)/L);
				phiContact = phi1 + phi2;

				robotFlipperAngles.flipperAngleRear = asin((pow(dRear,2) + pow(L,2) - pow(R,2))/ (2*L*dRear)) - atan(xRear/zRear) + asin((R-r)/L);
			}
			ROS_INFO("phi1 = %lf, phi2 = %lf, phiContact = %lf\n", phi1, phi2, phiContact);
			ROS_INFO("robotFlipperAngles.flipperAngleFront = %lf\n", robotFlipperAngles.flipperAngleFront);
		}
		else
		{
			robotFlipperAngles.flipperAngleRear = 85;
		}
	}
	else
	{
		robotFlipperAngles.flipperAngleRear = 0;
	}
}


geometry_msgs::Pose clcFlipperAngles::tfTransform(const geometry_msgs::Pose& pose,const std::string& destination_frame,const std::string& original_frame)
{
	// TF transformation of the Point which is nearest to the robot
	const ros::Time& scanTimeStamp = ros::Time (0);

    //std::cout<<"flipperToMapTransform"<<tfID<<std::endl;
	//ROS_INFO("pose:  x = %lf, y = %lf, z = %lf\n", pose.position.x, pose.position.y, pose.position.z);

    try
	{
		tfListener->waitForTransform (destination_frame, original_frame, scanTimeStamp, ros::Duration (3.0));
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}

	tf::Quaternion quat;
	quat.setX(pose.orientation.x);
	quat.setY(pose.orientation.y);
	quat.setZ(pose.orientation.z);
	quat.setW(pose.orientation.w);


	tf::Vector3 origin;

	origin.setX(pose.position.x);
	origin.setY(pose.position.y);
	origin.setZ(pose.position.z);

	tf::StampedTransform transform;

	tf::Stamped<tf::Pose> transPose (tf::Pose( quat,origin), scanTimeStamp, original_frame);

	tf::Stamped<tf::Pose> poseTransformed;

	try
	{
		tfListener->transformPose(destination_frame, scanTimeStamp, transPose, original_frame, poseTransformed);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR(" Point xyz %s", ex.what ());
	}
	origin=poseTransformed.getOrigin();
	quat=poseTransformed.getRotation();

	geometry_msgs::Pose returnPose;
	returnPose.position.x=origin.getX();
	returnPose.position.y=origin.getY();
	returnPose.position.z=origin.getZ();
	returnPose.orientation.x = quat.x();
	returnPose.orientation.y = quat.y();
	returnPose.orientation.z = quat.z();
	returnPose.orientation.w = quat.w();
	//ROS_INFO("returnPose:  x = %lf, y = %lf, z = %lf\n", returnPose.position.x, returnPose.position.y, returnPose.position.z);

	return returnPose;
}*/
