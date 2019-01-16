/*
 * approachPoint.h
 *
 *  Created on: Mar 28, 2018
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_APPROACHPOINT_H_
#define ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_APPROACHPOINT_H_
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <get_std_msgs/RegionBool.h>
#include "ContourPathPlanner.h"

class ApproachPoint {
public:
	ApproachPoint();
	virtual ~ApproachPoint ();
private:
	void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& pointcloud);

	bool selectedRegioncallBack(get_std_msgs::RegionBool::Request &req,
			get_std_msgs::RegionBool::Response &res);
	void getRealPositiomFromImage(const int& x,const int& y, pose& Pose);
    void xyzTfTransformation(pose& Pose);

	ros::NodeHandle nh;
    ros::ServiceServer selectRegion;
    ros::Subscriber subPointCloud;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZ;
	int mousex;
	int mousey;
	bool serviceCalled;
	bool approachPoint;

	// Tf Transformation
	const std::string BASE_FRAME = "base_link";
	//const std::string ODOM_FRAME = "odom";
	const std::string ODOM_FRAME = "map";

	ContourPathPlanner contourPathPlanner;
	std::unique_ptr<tf::TransformListener> tfListener;
	tf::StampedTransform transform;

};


#endif /* ROS_ROBOCUP_ROBOT_NAVIGATION_SRC_INCLUDE_APPROACHPOINT_H_ */
