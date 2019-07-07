/*
 * GazebObjectControl.cpp
 *
 *  Created on: 04.07.2019
 *      Author: chfo
 */

#include "traversability_estimation/GazebObjectControl.h"

#include <image_transport/image_transport.h>

#include <chrono>
#include <thread>
using namespace std;

GazebObjectControl::GazebObjectControl(ros::NodeHandle& nodeHandle)
		: nodeHandle_(nodeHandle)
{
	//ofstream myfile;
	//path = ros::package::getPath("traversability_estimation")+"/mazegenerator-master";
	//path = "/scratch-local/cdtemp/chfo/traverability/traversability_estimation/Gazebo Script";
	//path = "/home/chfo/Dropbox/Masterarbeit/python code/traversability_estimation/Gazebo Script";
	BASE_FRAME = "/base_link";
	MAP_FRAME = "/map";
	ODOM_FRAME = "/odom";
	tf_prefix = "//GETjag2";

	//tf_prefix = ros::this_node::getNamespace();
	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	std::istringstream iss (tf_prefix.substr(6, tf_prefix.size()));
	int robot_number = 1;
	iss >> robot_number;

	BASE_FRAME = tf_prefix + BASE_FRAME;
	MAP_FRAME = tf_prefix + MAP_FRAME;
	ODOM_FRAME = tf_prefix + ODOM_FRAME;

	modelPath = "/eda/gazebo/models/";

	setModelClient = nodeHandle_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	spawnModelClient = nodeHandle_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
	deleteModelClient = nodeHandle_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
	markerPublisher = nodeHandle_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 20);
	//resetRobot = nodeHandle_.advertiseService("reset_robot", &GazebObjectControl::resetRobotSrv, this);
	resetRobot = nodeHandle_.advertiseService("/" + tf_prefix+"/reset_robot", &GazebObjectControl::resetRobotSrv, this);

	goalPosePublischer = nodeHandle_.advertise<nav_msgs::Odometry>("/" + tf_prefix+"/goal_pose", 20);
	//goalPosePublischer = nodeHandle_.advertise<nav_msgs::Odometry>("goal_pose", 20);
	elevationMapImagePublisher = nodeHandle_.advertise<sensor_msgs::Image>("/" + tf_prefix+"/elevation_robot_ground_map", 20);
	//elevationMapImagePublisher = nodeHandle_.advertise<sensor_msgs::Image>("elevation_robot_ground_map", 20);

	static image_transport::ImageTransport it(nodeHandle_);
	static image_transport::Subscriber it_sub;
	//it_sub = it.subscribe("elevation_map_image", 1, boost::bind (&GazebObjectControl::MapImageCallback, this, _1));
	it_sub = it.subscribe("/" + tf_prefix+"/elevation_map_image", 1, boost::bind (&GazebObjectControl::MapImageCallback, this, _1));


	tfListener = unique_ptr<tf::TransformListener> (new tf::TransformListener);

	gazeboMoveObjectFrame = 'world';


	objects.push_back("object_cube_multicolor");
	objects.push_back("object_cylinder_multicolor");
	objects.push_back("object_robocup_box");
	objects.push_back("object_robocup_stairs");
	objects.push_back("object_robocup_stepfield");

	generateWorld(2, 5);

	std::this_thread::sleep_for(std::chrono::seconds(5));

	destroyWorld();

}

GazebObjectControl::~GazebObjectControl ()
{
}


void GazebObjectControl::publischGoal(const geometry_msgs::Pose& goalPose)
{

	nav_msgs::Odometry goalPoseMsg;
	//robotGoalPose = mapToBaseLinkTransform(goalPose);
	poseToOdomMsg(goalPose,goalPoseMsg);
	markerArray.markers.push_back (createMarker(tf_prefix+" Goal Pose", 1, goalPose.position.x, goalPose.position.y, 1.0, 1.0, 0.0,1.0));
	markerPublisher.publish(markerArray);
	goalPosePublischer.publish(goalPoseMsg);
	if(mapImageSet)
	{


		cv_ptr->image = getContactPoints.getRobotGroundImage(globalMapImage,1.5,1.5,  MAP_FRAME, BASE_FRAME);

		sensor_msgs::Image pubImage;
		cv_ptr->toImageMsg(pubImage);

		elevationMapImagePublisher.publish(pubImage);
	}


}

void GazebObjectControl::setObject(const string& modelName, geometry_msgs::Pose startPose)
{
	gazebo_msgs::SetModelState setmodelstate;
	gazebo_msgs::ModelState modelstate;
	modelstate.model_name = modelName;
	modelstate.reference_frame = "";

	geometry_msgs::Twist model_twist;
	model_twist.linear.x = 0;
	model_twist.linear.y = 0;
	model_twist.linear.z = 0;
	model_twist.angular.x = 0;
	model_twist.angular.y = 0.0;
	model_twist.angular.z = 0.0;
	modelstate.twist = model_twist;

	startPose = tfTransform(startPose, MAP_FRAME,ODOM_FRAME);

	modelstate.pose = startPose;

	setmodelstate.request.model_state = modelstate;

	if (setModelClient.call(setmodelstate))
	{
		ROS_INFO("Successful to call service: Set Model: %s",modelName.c_str());
	}
	else
	{
		ROS_ERROR("Successful to call service: Set Model: %s",modelName.c_str());
		//return 1;
	}
}

void GazebObjectControl::destroyWorld()
{
	for(auto object:spwanedObjects)
	{
		deleteObject(object);
	}
}

void GazebObjectControl::generateWorld(int minObjects, int maxObjects)
{
	random_device rd;
	mt19937 mt(rd());
	uniform_real_distribution<double> objectRndNumber(0, objects.size());
	uniform_real_distribution<double> anzObjectRndNumber(minObjects, maxObjects);

	int anzObjects = anzObjectRndNumber(mt);
	cout<<anzObjects<<endl;

	for(int i=0; i<anzObjects; i++)
	{
		string object = objects[objectRndNumber(mt)];
		cout<<object<<endl;
		geometry_msgs::Pose position = setRandomObst(false, false);
		spwanedObjects.push_back(object + "_" + std::to_string(i));
		spwanObject(object + "_" + std::to_string(i), object, position);
	}
}
geometry_msgs::Pose GazebObjectControl::setRandomObst(bool rotation, bool xyShift)
{
	geometry_msgs::Pose pose;

	random_device rd;
	mt19937 mt(rd());
	uniform_real_distribution<double> xPosition(2, 15);
	uniform_real_distribution<double> yPosition(-1, 1);
	uniform_real_distribution<double> zPosition(0, 0.3);
	uniform_real_distribution<double> rp_orientaton(M_PI/2, M_PI/2 + M_PI/4);
	uniform_real_distribution<double> y_orientaton(-M_PI, M_PI);


	pose.position.x = xPosition(mt);
	pose.position.y = 0;
	pose.position.z = 0;

	if(xyShift)
	{
		pose.position.y = yPosition(mt);
		pose.position.z = zPosition(mt);
	}

	tf2::Quaternion myQuaternion;

	if(rotation)
	{
		myQuaternion.setRPY(  rp_orientaton(mt),  rp_orientaton(mt), y_orientaton(mt));
	}
	else
	{
		myQuaternion.setRPY(0,0,0);
	}


	pose.orientation.x = myQuaternion.x();
	pose.orientation.y = myQuaternion.y();
	pose.orientation.z = myQuaternion.z();
	pose.orientation.w = myQuaternion.w();

	return pose;
}

void GazebObjectControl::spwanObject(const string& modelName, const string& xmlName, geometry_msgs::Pose startPose)
{
	startPose = tfTransform(startPose, MAP_FRAME,ODOM_FRAME);

	gazebo_msgs::SpawnModel spawnModel;

	spawnModel.request.model_name = modelName;

	string xmlFile = readXmlFile(xmlName);
	spawnModel.request.model_xml = xmlFile;
	spawnModel.request.robot_namespace = "";
	spawnModel.request.initial_pose = startPose;
	spawnModel.request.reference_frame = "";

	if (spawnModelClient.call(spawnModel))
	{
		ROS_INFO("Successful to call service: Spawn Model: %s",modelName.c_str());
	}
	else
	{
		ROS_ERROR("Successful to call service: Spawn Model: %s",modelName.c_str());
		//return 1;
	}
}

void GazebObjectControl::deleteObject(const string& modelName)
{
	gazebo_msgs::DeleteModel deleteModel;

	deleteModel.request.model_name = modelName;


	if (deleteModelClient.call(deleteModel))
	{
		ROS_INFO("Successful to call service: Delete Model: %s",modelName.c_str());
	}
	else
	{
		ROS_ERROR("Successful to call service: Delete Model: %s",modelName.c_str());
		//return 1;
	}
}

string  GazebObjectControl::readXmlFile(const string& name)
{
	string filePath = modelPath + name + "/model.sdf";

	ifstream t(filePath);
	string xmlText((std::istreambuf_iterator<char>(t)),
	                 std::istreambuf_iterator<char>());
	return xmlText;
}


void GazebObjectControl::poseToOdomMsg(const geometry_msgs::Pose& pose, nav_msgs::Odometry& setPose)
{
	 setPose.pose.pose.position.x = pose.position.x;
	 setPose.pose.pose.position.y = pose.position.y;
	 setPose.pose.pose.position.z = pose.position.z;

	 setPose.pose.pose.orientation.x =  pose.orientation.x;
	 setPose.pose.pose.orientation.y =  pose.orientation.y;
	 setPose.pose.pose.orientation.z =  pose.orientation.z;
	 setPose.pose.pose.orientation.w =  pose.orientation.w;
}


visualization_msgs::Marker GazebObjectControl::createMarker (std::string ns, int id, double x, double y,  double r = 1.0, double g = 0.0, double b = 0.0, double a = 1.0)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = MAP_FRAME;
	marker.header.stamp = ros::Time();
	marker.ns = ns;

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(0);

	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a; // Don't forget to set the alpha!

    marker.id = id;

	marker.pose.position.x = x;
	marker.pose.position.y = y;
//printf("marker.pose.position.x %lf, marker.pose.position.y %lf: \n",marker.pose.position.x, marker.pose.position.y);

	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	return marker;
}
bool GazebObjectControl::resetRobotSrv(std_srvs::Empty::Request &req,
		std_srvs::Empty::Response &res)
{
	geometry_msgs::Pose startPose;
	startPose.position.x = 0;
	startPose.position.y = 0;
	startPose.position.z = 0;

	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0, 0, 0);

	startPose.orientation.x = myQuaternion.x();
	startPose.orientation.y = myQuaternion.y();
	startPose.orientation.z = myQuaternion.z();
	startPose.orientation.w = myQuaternion.w();

	setObject(tf_prefix,startPose);
	return true;
}


geometry_msgs::Pose GazebObjectControl::tfTransform(const geometry_msgs::Pose& pose,const string& destination_frame,const string& original_frame)
{
	// TF transformation of the Point which is nearest to the robot
	const ros::Time& scanTimeStamp = ros::Time (0);

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

	return returnPose;
}


void GazebObjectControl::MapImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, "8UC1");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	(cv_ptr->image).copyTo(globalMapImage);


	mapImageSet = true;
}



