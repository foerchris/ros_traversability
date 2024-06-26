/*
 * GazebObjectControl.cpp
 *
 *  Created on: 04.07.2019
 *      Author: chfo
 */

#include "traversability_estimation/GazebObjectControl.h"

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;

GazebObjectControl::GazebObjectControl(ros::NodeHandle& nodeHandle)
		: nodeHandle_(nodeHandle),
		  getObjectInfoFromYaml_(nodeHandle)
{

	BASE_FRAME = "/base_link";
	MAP_FRAME = "/map";
	ODOM_FRAME = "/odom";

	tf_prefix = ros::this_node::getNamespace();
	if(tf_prefix == "/")
	{
		tf_prefix = "//GETjag1";
	}
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
	clcPathClient = nodeHandle_.serviceClient<std_srvs::Empty>("clc_path_to_goal");

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

	msg_timer = nodeHandle_.createTimer(ros::Duration(0.1), boost::bind (&GazebObjectControl::publischGoal, this, _1));
	reset_timer = nodeHandle_.createTimer(ros::Duration(0.1), boost::bind (&GazebObjectControl::resetCallback, this, _1));

	tfListener = unique_ptr<tf::TransformListener> (new tf::TransformListener);

	gazeboMoveObjectFrame = 'world';

	objects.clear();

	getObjectInfoFromYaml_.loadYaml("drl_flipper_objects");


	generateWorld2();

	goalPose.position.x = 18;
	goalPose.position.y = 0;
	goalPose.position.z = 0;

	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0, 0, 0);

	goalPose.orientation.x = myQuaternion.x();
	goalPose.orientation.y = myQuaternion.y();
	goalPose.orientation.z = myQuaternion.z();
	goalPose.orientation.w = myQuaternion.w();

	resetWorld = true;
	calculating = false;
}

GazebObjectControl::~GazebObjectControl ()
{
	destroyWorld();
}

void GazebObjectControl::resetCallback(const ros::TimerEvent& event)
{
	creatEnviroment();
}


void GazebObjectControl::creatEnviroment()
{
	if(resetWorld)
	{
		/*nodeHandle_.setParam("End_of_episode",false);
		resetWorld = false;
		// set all getjag to a zero pose
		setRobotZeroPose();

		//reset obstacles
		destroyWorld();

		generateWorld(1,4);

		nodeHandle_.setParam("reset_elevation_map",true);

		std::this_thread::sleep_for(std::chrono::seconds(3));

		clcGoalPathSrvsCall();

		nodeHandle_.setParam("Ready_to_Start_DRL_Agent",true);*/

		nodeHandle_.setParam("End_of_episode",false);
		resetWorld = false;
		// set all getjag to a zero pose
		setRobotZeroPose();
		std::this_thread::sleep_for(std::chrono::milliseconds(200));

		//reset obstacles
		resetAllObjects();

		setObjectInWorld();

		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		nodeHandle_.setParam("reset_elevation_map",true);

		std::this_thread::sleep_for(std::chrono::seconds(4));

		clcGoalPathSrvsCall();

		nodeHandle_.setParam("Ready_to_Start_DRL_Agent",true);


	}

	if(nodeHandle_.hasParam( "End_of_episode"))
	{
		nodeHandle_.getParam( "End_of_episode",resetWorld);
	}

}

void GazebObjectControl::clcGoalPathSrvsCall()
{

	std_srvs::Empty empty;
	int i =0;
	while (!clcPathClient.call(empty))
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		i+=1;
		if(i>=15)
		{
			break;
		}
	}/*
	else
	{
		ROS_ERROR("Unsuccessful to call service: clc_path_to_goal");
	}*/

}

void GazebObjectControl::publischGoal(const ros::TimerEvent& bla)
//void GazebObjectControl::publischGoal(const geometry_msgs::Pose& goalPose)
{


	nav_msgs::Odometry goalPoseMsg;
	geometry_msgs::Pose robotGoalPose = tfTransform(goalPose, BASE_FRAME, MAP_FRAME);
	poseToOdomMsg(robotGoalPose,goalPoseMsg);

	markerArray.markers.push_back (createMarker(tf_prefix+" Goal Pose", 1, goalPose.position.x, goalPose.position.y, 1.0, 1.0, 0.0,1.0));
	markerPublisher.publish(markerArray);
	goalPosePublischer.publish(goalPoseMsg);



	if(mapImageSet && globalMapImage.cols>0 && globalMapImage.rows>0)
	{
		cv::Mat depth( globalMapImage.rows, globalMapImage.cols, CV_32FC1 );
		cv::Mat alpha( globalMapImage.rows, globalMapImage.cols, CV_32FC1 );
		cv::Mat out[] = { depth,alpha };
		int from_to[] = { 0,0, 1,1};
		cv::mixChannels( &globalMapImage, 1, out, 2, from_to, 2 );

		cv::Mat groundImageDepth = getContactPoints.getRobotGroundImage(depth,2.2,1.5,  MAP_FRAME, BASE_FRAME);
		cv::Mat groundImageAlpha = getContactPoints.getRobotGroundImage(alpha,2.2,1.5,  MAP_FRAME, BASE_FRAME);

		//cv::mixChannels( &groundImage, 1, out, 2, from_to, 2 );
		cv::Mat depthAlpha[2];


		depthAlpha[0] = groundImageDepth;
		depthAlpha[1] = groundImageAlpha;
		cv::Mat image2 ;

		cv::merge(depthAlpha, 2, image2);


		image2.copyTo(cv_ptr->image);

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

	startPose = tfTransform(startPose, ODOM_FRAME, MAP_FRAME);

	modelstate.pose = startPose;

	setmodelstate.request.model_state = modelstate;

	if (setModelClient.call(setmodelstate))
	{
		//ROS_INFO("Successful to call service: Set Model: %s",modelName.c_str());
	}
	else
	{
		//ROS_ERROR("Successful to call service: Set Model: %s",modelName.c_str());
		//return 1;
	}
}

void GazebObjectControl::destroyWorld()
{
	for(auto object:spwanedObjects)
	{
		deleteObject(object.name);
		deleteObject(object.name +  "_mirror_");

	}
}

void GazebObjectControl::generateWorld(int minObjects, int maxObjects)
{

	int anzDifObjects = getObjectInfoFromYaml_.numPossibleObjects();
	random_device rd;
	mt19937 mt(rd());
	uniform_int_distribution<int> objectRndNumber(0, anzDifObjects-1);
	uniform_int_distribution<int> anzObjectRndNumber(minObjects, maxObjects);
	spwanedObjects.clear();

	int anzObjects = anzObjectRndNumber(mt);
	double lastX =0;
	for(int i=0; i<anzObjects; i++)
	{
		int randNum = objectRndNumber(mt);
		//int randNum = 0;

		string object = getObjectInfoFromYaml_.getType(randNum);
		string object_name = getObjectInfoFromYaml_.getName(randNum);

		object_options objectOptions;
		getObjectInfoFromYaml_.getinitPose(randNum,objectOptions);

		geometry_msgs::Pose position = setRandomObst(objectOptions,false,lastX);

		objectNameIndex objectName;
		objectName.name = object_name + "_" + tf_prefix + "_" + std::to_string(i);
		objectName.yamlIndex = i;
		spwanedObjects.push_back(objectName);
		spwanObject(objectName.name, object, position);
		if(objectOptions.mirrorObject)
		{
			lastX = position.position.x;
			geometry_msgs::Pose position = setRandomObst(objectOptions,true,lastX);
			objectName.name = object_name + "_" + tf_prefix + "_mirror_" + std::to_string(i);
			objectName.yamlIndex = i;
			spwanedObjects.push_back(objectName);
			spwanObject(objectName.name, object, position);
		}
	}

}


void GazebObjectControl::generateWorld2()
{
	geometry_msgs::Pose pose;
	pose.position.x = 0;
	pose.position.y = 10;
	pose.position.z = 0;

	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0, 0, 0);

	pose.orientation.x = myQuaternion.x();
	pose.orientation.y = myQuaternion.y();
	pose.orientation.z = myQuaternion.z();
	pose.orientation.w = myQuaternion.w();

	for(int i=0; i<getObjectInfoFromYaml_.numPossibleObjects(); i++)
	{
		for(int j=0; j<getObjectInfoFromYaml_.numThisObjects(i); j++)
		{

			string object = getObjectInfoFromYaml_.getType(i);
			string object_name = getObjectInfoFromYaml_.getName(i);
			object_options objectOptions;
			getObjectInfoFromYaml_.getinitPose(i,objectOptions);

			//geometry_msgs::Pose position = setRandomObst(objectOptions,false,lastX);

			objectNameIndex objectName;
			objectName.name = object_name + "_" + tf_prefix + "_" + std::to_string(j);
			objectName.yamlIndex = i;

			spwanedObjects.push_back(objectName);
			spwanObject(objectName.name, object, pose);
			if(objectOptions.mirrorObject)
			{
			//lastX = position.position.x;
				//geometry_msgs::Pose position = setRandomObst(objectOptions,true,lastX);
				string objectName = object_name + "_" + tf_prefix + "_" + std::to_string(j)+ "_mirror_";

				//spwanedObjects.push_back(objectName);
				spwanObject(objectName, object, pose);
			}
		}
	}
}
void GazebObjectControl::setObjectInWorld()
{
	random_device rd;
	mt19937 mt(rd());

	uniform_int_distribution<int> randNumber(1 , spwanedObjects.size());
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	int numberOfObjects = randNumber(mt);

	double lastX =0;

	std::shuffle(spwanedObjects.begin(), spwanedObjects.end(), std::default_random_engine(seed));

	for(int i=0; i<numberOfObjects; i++)
	{
		string object = getObjectInfoFromYaml_.getType(spwanedObjects[i].yamlIndex);
		std::cout<<"object"<<object<<std::endl;


		object_options objectOptions;
		getObjectInfoFromYaml_.getinitPose(spwanedObjects[i].yamlIndex,objectOptions);

		double _;
		geometry_msgs::Pose position = setRandomObst(objectOptions,false,_);
		setObject(spwanedObjects[i].name, position);
		lastX = position.position.x;

		if(objectOptions.mirrorObject)
		{
			position = setRandomObst(objectOptions,true,lastX);
			setObject(spwanedObjects[i].name + "_mirror_", position);
		}

	}
}
geometry_msgs::Pose GazebObjectControl::setRandomObst(const object_options& objectOptions,const bool& mirror, const double& lastX)
{
	geometry_msgs::Pose pose;



	random_device rd;
	mt19937 mt(rd());

	pose.position.x = creatRndPosition(objectOptions.x);
	pose.position.y = creatRndPosition(objectOptions.y);
	pose.position.z = creatRndPosition(objectOptions.z);


	tf2::Quaternion myQuaternion;
	double yaw = creatRndPosition(objectOptions.yaw);


	if(mirror)
	{
		pose.position.x = lastX + objectOptions.length;
		yaw = yaw - M_PI;
	}

	myQuaternion.setRPY(creatRndPosition(objectOptions.roll),  creatRndPosition(objectOptions.pitch), yaw);


	pose.orientation.x = myQuaternion.x();
	pose.orientation.y = myQuaternion.y();
	pose.orientation.z = myQuaternion.z();
	pose.orientation.w = myQuaternion.w();



	return pose;
}

void GazebObjectControl::resetAllObjects()
{
	geometry_msgs::Pose pose;
	pose.position.x = 0;
	pose.position.y = 10;
	pose.position.z = 0;

	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0, 0, 0);

	pose.orientation.x = myQuaternion.x();
	pose.orientation.y = myQuaternion.y();
	pose.orientation.z = myQuaternion.z();
	pose.orientation.w = myQuaternion.w();

	for(auto object:spwanedObjects)
	{
		object_options objectOptions;
		getObjectInfoFromYaml_.getinitPose(object.yamlIndex,objectOptions);
		setObject(object.name, pose);
		if(objectOptions.mirrorObject)
		{
			setObject(object.name + "_mirror_", pose);
		}
	}
}

double GazebObjectControl::creatRndPosition(const min_max_object_pose& minMaxObjectPose)
{
	random_device rd;
	mt19937 mt(rd());
	double value;
	if(minMaxObjectPose.minMaxcase == 1)
	{
		uniform_real_distribution<double> xPosition(minMaxObjectPose.min, minMaxObjectPose.max);
		value = xPosition(mt);
	}
	else if(minMaxObjectPose.minMaxcase == 2)
	{
		value = minMaxObjectPose.min;
	}
	else
	{
		value = minMaxObjectPose.min;
	}
	return value;

}
void GazebObjectControl::spwanObject(const string& modelName, const string& xmlName, geometry_msgs::Pose startPose)
{
	startPose = tfTransform(startPose, ODOM_FRAME, MAP_FRAME);

	gazebo_msgs::SpawnModel spawnModel;

	spawnModel.request.model_name = modelName;

	string xmlFile = readXmlFile(xmlName);
	spawnModel.request.model_xml = xmlFile;
	spawnModel.request.robot_namespace = "";
	spawnModel.request.initial_pose = startPose;
	spawnModel.request.reference_frame = "";

	if (spawnModelClient.call(spawnModel))
	{
		//ROS_INFO("Successful to call service: Spawn Model: %s",modelName.c_str());
	}
	else
	{
		//ROS_ERROR("Successful to call service: Spawn Model: %s",modelName.c_str());
		//return 1;
	}
}

void GazebObjectControl::deleteObject(const string& modelName)
{
	gazebo_msgs::DeleteModel deleteModel;

	deleteModel.request.model_name = modelName;


	if (deleteModelClient.call(deleteModel))
	{
		//ROS_INFO("Successful to call service: Delete Model: %s",modelName.c_str());
	}
	else
	{
		//ROS_ERROR("Successful to call service: Delete Model: %s",modelName.c_str());
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
	setRobotZeroPose();
	return true;
}
void GazebObjectControl::setRobotZeroPose()
{
	geometry_msgs::Pose startPose;
	startPose.position.x = 0;
	startPose.position.y = 0;
	startPose.position.z = 0.5;

	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0, 0, 0);

	startPose.orientation.x = myQuaternion.x();
	startPose.orientation.y = myQuaternion.y();
	startPose.orientation.z = myQuaternion.z();
	startPose.orientation.w = myQuaternion.w();

	setObject(tf_prefix,startPose);
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
		cv_ptr = cv_bridge::toCvCopy(msg, "32FC2");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat image;
	(cv_ptr->image).copyTo(image);

	(image).copyTo(globalMapImage);



	mapImageSet = true;
}



