/*
 * approachPoint.cpp
 *
 *  Created on: Mar 28, 2018
 *      Author: chfo
 */
#include "ApproachPoint.h"
ApproachPoint::ApproachPoint()
{
	cloudXYZ=nullptr;
	mousex=0;
	mousey=0;
	serviceCalled=false;
	approachPoint=false;
	tfListener = std::unique_ptr<tf::TransformListener> (new tf::TransformListener);
	selectRegion  = nh.advertiseService("select_region", &ApproachPoint::selectedRegioncallBack, this);
	subPointCloud  = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGBA> > ("xtion/depth/points", 2, &ApproachPoint::pointCloudCallback,this);


}
ApproachPoint::~ApproachPoint()
{

}

void ApproachPoint::pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& pointcloud)
{
	double planningRadius=0.5;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>(* pointcloud));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
	copyPointCloud(*cloud,*cloud_xyz);
	std::vector<pose> Poses;

	contourPathPlanner.forwardBackwarMode(true,1,1);
	std::cout<<"serviceCalled"<<serviceCalled<<std::endl;

	if(serviceCalled)
	{
		cloudXYZ=cloud;
		pose Pose;
		std::cout<<"mousex"<<mousex<<std::endl;

		getRealPositiomFromImage(mousex,mousey, Pose);
		Pose.yaw=0;
		xyzTfTransformation(Pose);
		Poses.push_back(Pose);
		contourPathPlanner.setPathPoints(Poses);
		contourPathPlanner.clcPathToSinglePoint(planningRadius);
		contourPathPlanner.followPath(ReferencePathControl,true);
		approachPoint=true;
		serviceCalled=false;
	}
	if(approachPoint)
	{
		//std::cout<<"ReferencePathControl"<<ReferencePathControl.at(0).x<<std::endl;
		if(!contourPathPlanner.followPath(ReferencePathControl,false))
		{
			approachPoint=false;
		}
	}
}

void ApproachPoint::getRealPositiomFromImage(const int& x,const int& y, pose& Pose)
{
	Pose.x =cloudXYZ->points[ y*cloudXYZ->width+x].x;
	Pose.y =cloudXYZ->points[ y*cloudXYZ->width+x].y;
	Pose.z =cloudXYZ->points[ y*cloudXYZ->width+x].z;
}

void ApproachPoint::xyzTfTransformation(pose& Pose)
{
	// TF transformation of the Point which is nearest du the robot
	const std::string& scanFrameID = cloudXYZ->header.frame_id;
	const ros::Time& scanTimeStamp = ros::Time (0);
	tfListener->waitForTransform (BASE_FRAME, scanFrameID, scanTimeStamp, ros::Duration (2.0));

	tf::Stamped<tf::Point> point (tf::Point(Pose.x, Pose.y, Pose.z), scanTimeStamp, scanFrameID);
	tf::Stamped<tf::Point> pointTransformed;
	try
	{
		tfListener->transformPoint (BASE_FRAME, scanTimeStamp, point, scanFrameID, pointTransformed);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR (" Point xyz %s", ex.what ());
		return;
	}
	Pose.x = pointTransformed.getX ();
	Pose.y = pointTransformed.getY ();
	Pose.z = pointTransformed.getZ ();

}

bool ApproachPoint::selectedRegioncallBack(get_std_msgs::RegionBool::Request &req,
		get_std_msgs::RegionBool::Response &res)
{
	mousex = req.x;
	mousey = req.y;
	serviceCalled=true;
	return true;

}


