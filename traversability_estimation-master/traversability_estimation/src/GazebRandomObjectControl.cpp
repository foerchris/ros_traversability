/*
 * GazebRandomObjectControl.cpp
 *
 *  Created on: 04.07.2019
 *      Author: chfo
 */

#include "traversability_estimation/GazebRandomObjectControl.h"

#include <image_transport/image_transport.h>


using namespace std;

GazebRandomObjectControl::GazebRandomObjectControl(ros::NodeHandle& nodeHandle)
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

	//resetRobot = nodeHandle_.advertiseService("reset_robot", &GazebRandomObjectControl::resetRobotSrv, this);
	resetRobot = nodeHandle_.advertiseService("/" + tf_prefix+"/reset_robot", &GazebRandomObjectControl::resetRobotSrv, this);

	goalPosePublischer = nodeHandle_.advertise<nav_msgs::Odometry>("/" + tf_prefix+"/goal_pose", 20);
	//goalPosePublischer = nodeHandle_.advertise<nav_msgs::Odometry>("goal_pose", 20);
	elevationMapImagePublisher = nodeHandle_.advertise<sensor_msgs::Image>("/" + tf_prefix+"/elevation_robot_ground_map", 20);
	//elevationMapImagePublisher = nodeHandle_.advertise<sensor_msgs::Image>("elevation_robot_ground_map", 20);

	static image_transport::ImageTransport it(nodeHandle_);
	static image_transport::Subscriber it_sub;
	//it_sub = it.subscribe("elevation_map_image", 1, boost::bind (&GazebRandomObjectControl::MapImageCallback, this, _1));
	it_sub = it.subscribe("/" + tf_prefix+"/elevation_map_image", 1, boost::bind (&GazebRandomObjectControl::MapImageCallback, this, _1));

	msg_timer = nodeHandle_.createTimer(ros::Duration(0.1), boost::bind (&GazebRandomObjectControl::publischGoal, this, _1));
	reset_timer = nodeHandle_.createTimer(ros::Duration(0.1), boost::bind (&GazebRandomObjectControl::resetCallback, this, _1));

	tfListener = unique_ptr<tf::TransformListener> (new tf::TransformListener);

	gazeboMoveObjectFrame = 'world';

	objects.clear();

	getObjectInfoFromYaml_.loadYaml("drl_objects");

	generateWorld();


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

GazebRandomObjectControl::~GazebRandomObjectControl ()
{
	destroyWorld();
}

void GazebRandomObjectControl::resetCallback(const ros::TimerEvent& event)
{
	creatEnviroment();
}


void GazebRandomObjectControl::creatEnviroment()
{
	if(resetWorld)
	{
		mazeReader.reset();
		nodeHandle_.setParam("End_of_episode",false);
		resetWorld = false;
		// set all getjag to a zero pose
		setRobotZeroPose();

		//reset obstacles
		resetAllObjects();
		setObjectInWorld();

		setRobotStartPose();

		goalPose = transformMaze(mazeReader.getRandomCell());

		nodeHandle_.setParam("reset_elevation_map",true);

		std::this_thread::sleep_for(std::chrono::seconds(3));

		nodeHandle_.setParam("Ready_to_Start_DRL_Agent",true);

	}

	if(nodeHandle_.hasParam( "End_of_episode"))
	{
		nodeHandle_.getParam( "End_of_episode",resetWorld);
	}
}

void GazebRandomObjectControl::clcGoalPathSrvsCall()
{
	std_srvs::Empty empty;
	if (clcPathClient.call(empty))
	{
		ROS_INFO("Successful to call service: clc_path_to_goal");
	}
	else
	{
		ROS_ERROR("Unsuccessful to call service: clc_path_to_goal");
		//return 1;
	}

}

void GazebRandomObjectControl::publischGoal(const ros::TimerEvent& bla)
//void GazebRandomObjectControl::publischGoal(const geometry_msgs::Pose& goalPose)
{


	nav_msgs::Odometry goalPoseMsg;
	geometry_msgs::Pose robotGoalPose = tfTransform(goalPose, BASE_FRAME, MAP_FRAME);
	poseToOdomMsg(robotGoalPose,goalPoseMsg);

	markerArray.markers.push_back (createMarker(tf_prefix+" Goal Pose", 1, goalPose.position.x, goalPose.position.y, 1.0, 1.0, 0.0,1.0));
	markerPublisher.publish(markerArray);
	goalPosePublischer.publish(goalPoseMsg);

	if(mapImageSet && globalMapImage.cols>0 && globalMapImage.rows>0)
	{


		cv::Mat groundImage = getContactPoints.getRobotGroundImage(globalMapImage,2.2,1.5,  MAP_FRAME, BASE_FRAME);
		groundImage.convertTo(cv_ptr->image, CV_16UC1, 255.0 );

		sensor_msgs::Image pubImage;
		cv_ptr->toImageMsg(pubImage);

		elevationMapImagePublisher.publish(pubImage);
	}


}

void GazebRandomObjectControl::setObject(const string& modelName, geometry_msgs::Pose startPose)
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
		ROS_INFO("Successful to call service: Set Model: %s",modelName.c_str());
	}
	else
	{
		ROS_ERROR("Successful to call service: Set Model: %s",modelName.c_str());
		//return 1;
	}
}

void GazebRandomObjectControl::destroyWorld()
{
	for(auto object:spwanedObjects)
	{
		deleteObject(object.name);
	}
}

void GazebRandomObjectControl::resetAllObjects()
{
	geometry_msgs::Pose pose;
	pose.position.x = 7;
	pose.position.y = 0;
	pose.position.z = 0;

	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0, 0, 0);

	pose.orientation.x = myQuaternion.x();
	pose.orientation.y = myQuaternion.y();
	pose.orientation.z = myQuaternion.z();
	pose.orientation.w = myQuaternion.w();

	for(auto object:spwanedObjects)
	{
		setObject(object.name, goalPose);
	}
}

void GazebRandomObjectControl::setObjectInWorld()
{
	random_device rd;
	mt19937 mt(rd());

	uniform_int_distribution<int> randNumber(3 , spwanedObjects.size());
	uniform_int_distribution<int> randObject(0 , spwanedObjects.size());
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	int numberOfObjects = randNumber(mt);


	for(int i=0; i<numberOfObjects; i++)
	{

		std::shuffle(spwanedObjects.begin(), spwanedObjects.end(), std::default_random_engine(seed));
		string object = getObjectInfoFromYaml_.getName(spwanedObjects[i].yamlIndex);


		object_options objectOptions;
		getObjectInfoFromYaml_.getinitPose(spwanedObjects[i].yamlIndex,objectOptions);
		double _;
		geometry_msgs::Pose position = setRandomObst(objectOptions,false,_);

		setObject(spwanedObjects[i].name, position);

	}

}
void GazebRandomObjectControl::generateWorld()
{
	geometry_msgs::Pose pose;
	pose.position.x = 7;
	pose.position.y = 0;
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

			string object = getObjectInfoFromYaml_.getName(i);
			object_options objectOptions;
			getObjectInfoFromYaml_.getinitPose(i,objectOptions);

			//geometry_msgs::Pose position = setRandomObst(objectOptions,false,lastX);

			objectNameIndex objectName;
			objectName.name = object + "_" + tf_prefix + "_" + std::to_string(j);
			objectName.yamlIndex = i;

			spwanedObjects.push_back(objectName);
			spwanObject(objectName.name, object, pose);
		}
	}
}
geometry_msgs::Pose GazebRandomObjectControl::setRandomObst(const object_options& objectOptions,const bool& mirror, const double& lastX)
{
	geometry_msgs::Pose pose;


	maze randomCell = mazeReader.getRandomCell();
	pose = transformMaze(randomCell);
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

double GazebRandomObjectControl::creatRndPosition(const min_max_object_pose& minMaxObjectPose)
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
void GazebRandomObjectControl::spwanObject(const string& modelName, const string& xmlName, geometry_msgs::Pose startPose)
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
		ROS_INFO("Successful to call service: Spawn Model: %s",modelName.c_str());
	}
	else
	{
		ROS_ERROR("Successful to call service: Spawn Model: %s",modelName.c_str());
		//return 1;
	}
}

void GazebRandomObjectControl::deleteObject(const string& modelName)
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

string  GazebRandomObjectControl::readXmlFile(const string& name)
{
	string filePath = modelPath + name + "/model.sdf";

	ifstream t(filePath);
	string xmlText((std::istreambuf_iterator<char>(t)),
	                 std::istreambuf_iterator<char>());
	return xmlText;
}


void GazebRandomObjectControl::poseToOdomMsg(const geometry_msgs::Pose& pose, nav_msgs::Odometry& setPose)
{
	 setPose.pose.pose.position.x = pose.position.x;
	 setPose.pose.pose.position.y = pose.position.y;
	 setPose.pose.pose.position.z = pose.position.z;

	 setPose.pose.pose.orientation.x =  pose.orientation.x;
	 setPose.pose.pose.orientation.y =  pose.orientation.y;
	 setPose.pose.pose.orientation.z =  pose.orientation.z;
	 setPose.pose.pose.orientation.w =  pose.orientation.w;
}


visualization_msgs::Marker GazebRandomObjectControl::createMarker (std::string ns, int id, double x, double y,  double r = 1.0, double g = 0.0, double b = 0.0, double a = 1.0)
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
bool GazebRandomObjectControl::resetRobotSrv(std_srvs::Empty::Request &req,
		std_srvs::Empty::Response &res)
{
	setRobotZeroPose();
	return true;
}

void GazebRandomObjectControl::setRobotStartPose()
{
	geometry_msgs::Pose startPose;
	startPose = transformMaze(mazeReader.getRandomCell());
	startPose = creatRandomOrientation(startPose);

	setObject(tf_prefix,startPose);
}

void GazebRandomObjectControl::setRobotZeroPose()
{
	geometry_msgs::Pose startPose;
	startPose.position.x = 6;
	startPose.position.y = 6;
	startPose.position.z = 0.5;

	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0, 0, 0);

	startPose.orientation.x = myQuaternion.x();
	startPose.orientation.y = myQuaternion.y();
	startPose.orientation.z = myQuaternion.z();
	startPose.orientation.w = myQuaternion.w();

	setObject(tf_prefix,startPose);
}


geometry_msgs::Pose GazebRandomObjectControl::tfTransform(const geometry_msgs::Pose& pose,const string& destination_frame,const string& original_frame)
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


void GazebRandomObjectControl::MapImageCallback(const sensor_msgs::ImageConstPtr& msg)
{

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	//cv::imshow("bhaldhw", cv_ptr->image);
	//cv::waitKey(1);
	cv::Mat image;
	(cv_ptr->image).copyTo(image);
	image.convertTo(image, CV_32FC1, 1/255.0 );
	//cv::imshow("bhaldhw", image);
	//cv::waitKey(1);
	(image).copyTo(globalMapImage);


	mapImageSet = true;
}

geometry_msgs::Pose GazebRandomObjectControl::transformMaze(maze position)
{
	geometry_msgs::Pose pose;

	pose.position.x = position.x;

	pose.position.y = position.y;

	pose.position.z = 0;

	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0, 0, position.orientation/180*M_PI);

	pose.orientation.x = myQuaternion.x();
	pose.orientation.y = myQuaternion.y();
	pose.orientation.z = myQuaternion.z();
	pose.orientation.w = myQuaternion.w();
	return pose;
}


geometry_msgs::Pose GazebRandomObjectControl::creatRandomOrientation(geometry_msgs::Pose pose)
{
	std::random_device rd;
	std::mt19937 mt(rd());


	std::uniform_real_distribution<double> orientaton(-M_PI, M_PI);

	tf2::Quaternion myQuaternion;

	myQuaternion.setRPY( 0, 0, orientaton(mt));

	pose.orientation.x = myQuaternion.x();
	pose.orientation.y = myQuaternion.y();
	pose.orientation.z = myQuaternion.z();
	pose.orientation.w = myQuaternion.w();
	return pose;
}

