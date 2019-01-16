/*
 * robot_positioning.cpp
 *
 *  Created on: 10.10.2018
 *      Author: chfo
 */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetModelState.h>

#include <tf2/LinearMath/Quaternion.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sstream>
ros::Publisher goalPosePublischer;
ros::Publisher markerPublisher;

visualization_msgs::Marker createMarker (std::string ns, int id, double x, double y,  double r, double g , double b , double a);
void creatRandomPose(geometry_msgs::Pose& pose, int xyInterval);
void poseToOdomMsg(const geometry_msgs::Pose& pose, nav_msgs::Odometry& setPose);
void setObjectPose(const std::string& name, geometry_msgs::Pose startPose, gazebo_msgs::SetModelState& setmodelstate, bool robot, int area);
geometry_msgs::Pose globaleToRobotTransform(geometry_msgs::Pose pose);
void setGETjagZeroPose(const std::string& name, gazebo_msgs::SetModelState& setmodelstate);
void setPoseToParamServer(const ros::NodeHandle& nh, const geometry_msgs::Pose& pose);
void transformToArea(std::string name, geometry_msgs::Pose& pose);
std::string BASE_FRAME = "/base_link";
std::string MAP_FRAME = "/map";
std::string gazeboMoveObjectFrame = "world";
std::string tf_prefix = "GETjag";
std::unique_ptr<tf::TransformListener> tfListener;
tf::StampedTransform transform;

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_positioning");
	ros::NodeHandle nh;
	tf_prefix = ros::this_node::getNamespace();
	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	std::cout<<"tf_prefix"<<tf_prefix<<std::endl;
	BASE_FRAME = tf_prefix + BASE_FRAME;
	MAP_FRAME = tf_prefix + MAP_FRAME;

	ros::Rate rate (10);

	tfListener = std::unique_ptr<tf::TransformListener> (new tf::TransformListener);

	srand ( time(NULL) );

	ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

	goalPosePublischer = nh.advertise<nav_msgs::Odometry>("goal_pose", 20);
	markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 20);

	gazebo_msgs::SetModelState setmodelstate;
	geometry_msgs::Pose startPose;


	/*// Set Obstacles and Robots
	if(tf_prefix=="GETjag1")
	{
		for(int i=1;i<5;i++)
		{
			// set all getjag to a zero pose
			setGETjagZeroPose("GETjag"+std::to_string(i), setmodelstate);
			if (client.call(setmodelstate))
			{
				//ROS_INFO("Successful to call service: Set GETjag!!!");
			}
			else
			{
				ROS_ERROR("Failed to call service: Set GETjag!!!");
				return 1;
			}
		}
		rate.sleep();

		int possibleRandomObstacles = 5;
		int minObstacles = 3;
		int anzObstacles = (rand() % possibleRandomObstacles) + minObstacles;
		for(int i=1; i<=anzObstacles+1; i++)
		{
			creatRandomPose(startPose, 5);
			for(int j=1;j<5;j++)
			{
				// set obstacles
				setObjectPose("object_robocup_wall_clone_"+std::to_string(i+((j-1)*20)), startPose,setmodelstate, false,j);
				if (client.call(setmodelstate))
				{
					//ROS_INFO("Successful to call service: Set Obstacle: %i",i);
				}
				else
				{
					ROS_ERROR("Failed to call service: Set Obstacle: %i",i);
					return 1;
				}
			}
		}

		for(int i=1;i<5;i++)
		{
			// set all getjag to a random pose
			creatRandomPose(startPose, 4.5);
			setObjectPose("GETjag"+std::to_string(i),startPose,setmodelstate, true,i);
			if (client.call(setmodelstate))
			{
				//ROS_INFO("Successful to call service: Set GETjag!!!");
			}
			else
			{
				ROS_ERROR("Failed to call service: Set GETjag!!!");
				return 1;
			}
		}
	}
	else
	{

		// set getjag to a random pose
		creatRandomPose(startPose, 4.5);
		setObjectPose(tf_prefix, startPose, setmodelstate, true,1);
		if (client.call(setmodelstate))
		{
			//ROS_INFO("Successful to call service: Set GETjag!!!");
		}
		else
		{
			ROS_ERROR("Failed to call service: Set GETjag!!!");
			return 1;
		}


	}*/
	// set all getjag to a zero pose
	setGETjagZeroPose(tf_prefix, setmodelstate);
	if (client.call(setmodelstate))
	{
		//ROS_INFO("Successful to call service: Set GETjag!!!");
	}
	else
	{
		ROS_ERROR("Failed to call service: Set GETjag!!!");
		return 1;
	}

	rate.sleep();

	int possibleRandomObstacles = 5;
	int minObstacles = 3;
	int anzObstacles = (rand() % possibleRandomObstacles) + minObstacles;
	for(int i=1; i<=anzObstacles+1; i++)
	{
		creatRandomPose(startPose, 5);
		int j;
		if(tf_prefix == "GETjag1")
		{
			j=1;
		}
		else if(tf_prefix == "GETjag2")
		{
			j=2;
		}
		else if(tf_prefix == "GETjag3")
		{
			j=3;
		}
		else
		{
			j=4;
		}
		// set obstacles
		setObjectPose("object_robocup_wall_clone_"+std::to_string(i+(j*20)), startPose,setmodelstate, false,j);
		if (client.call(setmodelstate))
		{
			//ROS_INFO("Successful to call service: Set Obstacle: %i",i);
		}
		else
		{
			ROS_ERROR("Failed to call service: Set Obstacle: %i",i);
			return 1;
		}
	}
	// set getjag to a random pose
	creatRandomPose(startPose, 4.5);
	setObjectPose(tf_prefix, startPose, setmodelstate, true,1);
	if (client.call(setmodelstate))
	{
		//ROS_INFO("Successful to call service: Set GETjag!!!");
	}
	else
	{
		ROS_ERROR("Failed to call service: Set GETjag!!!");
		return 1;
	}

	// Create random goal position message
	geometry_msgs::Pose mapGoalPose;
	creatRandomPose(mapGoalPose, 4.5);
	transformToArea(tf_prefix,mapGoalPose);

	//setPoseToParamServer(nh , mapGoalPose);



	printf("mapGoalPose.position.x %lf, mapGoalPose.position.y %lf: \n",mapGoalPose.position.x, mapGoalPose.position.y);

	//printf("goalPose.position.x %lf, goalPose.position.y %lf: \n",goalPose.position.x, goalPose.position.y);


	visualization_msgs::MarkerArray markerArray;


	markerArray.markers.push_back (createMarker(tf_prefix+" Goal Pose", 1, mapGoalPose.position.x, mapGoalPose.position.y, 1.0, 1.0, 0.0,1.0));

	geometry_msgs::Pose robotGoalPose;

	nav_msgs::Odometry goalPoseMsg;

	while(ros::ok())
	{
	  robotGoalPose = globaleToRobotTransform(mapGoalPose);
	  poseToOdomMsg(robotGoalPose,goalPoseMsg);
	  markerPublisher.publish(markerArray);
	  goalPosePublischer.publish(goalPoseMsg);
	  ros::spinOnce ();
	  rate.sleep();
	}

	return 0;
}

void setPoseToParamServer(const ros::NodeHandle& nh, const geometry_msgs::Pose& pose)
{
	std::ostringstream strs;
	strs << pose.position.x;
	std::string str = strs.str();

	str.append(":");
	strs.clear();
	strs << pose.position.y;
	str.append(strs.str());

	str.append(":");
	strs.clear();
	strs << pose.position.z;
	str.append(strs.str());

	str.append(":");
	strs.clear();
	strs << pose.orientation.x;
	str.append(strs.str());

	str.append(":");
	strs.clear();
	strs << pose.orientation.y;
	str.append(strs.str());


	str.append(":");
	strs.clear();
	strs << pose.orientation.z;
	str.append(strs.str());

	str.append(":");
	strs.clear();
	strs << pose.orientation.w;
	str.append(strs.str());

	std::cout<<str<<std::endl;
	nh.setParam("GETjag/desired_robot_pos", str);

}

void transformToArea(std::string name, geometry_msgs::Pose& pose)
{
	if(name=="GETjag1")
	{
		pose.position.x = pose.position.x+5;
		pose.position.y = pose.position.y+5;
	}
	else if(name=="GETjag2")
	{
		pose.position.x = pose.position.x+5;
		pose.position.y = pose.position.y-5;

	}
	else if(name=="GETjag3")
	{
		pose.position.x = pose.position.x-5;
		pose.position.y = pose.position.y+5;

	}
	else
	{
		pose.position.x = pose.position.x-5;
		pose.position.y = pose.position.y-5;
	}
}


void creatRandomPose(geometry_msgs::Pose& pose, int xyInterval)
{
	int randomNumber = (rand() % (xyInterval*2000));
	pose.position.x = randomNumber/1000.0-xyInterval;

	randomNumber = (rand() % (xyInterval*2000));
	pose.position.y = randomNumber/1000.0-xyInterval;

	pose.position.z = 0;

	tf2::Quaternion myQuaternion;
	randomNumber = rand() % 1000;
	myQuaternion.setRPY( 0, 0, (randomNumber/1000.0 - 0.5)*M_PI );

	pose.orientation.x = myQuaternion.x();
	pose.orientation.y = myQuaternion.y();
	pose.orientation.z = myQuaternion.z();
	pose.orientation.w = myQuaternion.w();
}

void poseToOdomMsg(const geometry_msgs::Pose& pose, nav_msgs::Odometry& setPose)
{
	 setPose.pose.pose.position.x = pose.position.x;
	 setPose.pose.pose.position.y = pose.position.y;
	 setPose.pose.pose.position.z = pose.position.z;

	 setPose.pose.pose.orientation.x =  pose.orientation.x;
	 setPose.pose.pose.orientation.y =  pose.orientation.y;
	 setPose.pose.pose.orientation.z =  pose.orientation.z;
	 setPose.pose.pose.orientation.w =  pose.orientation.w;
}

void setGETjagZeroPose(const std::string& name, gazebo_msgs::SetModelState& setmodelstate)
{
	gazebo_msgs::ModelState modelstate;
	modelstate.model_name = name;
	modelstate.reference_frame = gazeboMoveObjectFrame;

	geometry_msgs::Twist model_twist;
	model_twist.linear.x = 0;
	model_twist.linear.y = 0;
	model_twist.linear.z = 0;
	model_twist.angular.x = 0;
	model_twist.angular.y = 0.0;
	model_twist.angular.z = 0.0;
	modelstate.twist = model_twist;

	geometry_msgs::Pose startPose;
	if(name=="GETjag1")
	{
		startPose.position.x = 1;
		startPose.position.y = 1;
	}
	else if(name=="GETjag2")
	{
		startPose.position.x = 1;
		startPose.position.y = -1;

	}
	else if(name=="GETjag3")
	{
		startPose.position.x = -1;
		startPose.position.y = 1;

	}
	else
	{
		startPose.position.x = -1;
		startPose.position.y = -1;

	}
	startPose.position.z = 3;



	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY( 0, 0, M_PI);

	startPose.orientation.x = myQuaternion.x();
	startPose.orientation.y = myQuaternion.y();
	startPose.orientation.z = myQuaternion.z();
	startPose.orientation.w = myQuaternion.w();

	modelstate.pose = startPose;

	setmodelstate.request.model_state = modelstate;
}


void setObjectPose(const std::string& name, geometry_msgs::Pose startPose, gazebo_msgs::SetModelState& setmodelstate, bool robot, int area)
{
	gazebo_msgs::ModelState modelstate;
	modelstate.model_name = name;
	modelstate.reference_frame = gazeboMoveObjectFrame;

	geometry_msgs::Twist model_twist;
	model_twist.linear.x = 0;
	model_twist.linear.y = 0;
	model_twist.linear.z = 0;
	model_twist.angular.x = 0;
	model_twist.angular.y = 0.0;
	model_twist.angular.z = 0.0;
	modelstate.twist = model_twist;

	if(robot)
	{
		transformToArea(name,startPose);
		startPose.position.z = 2;
	}
	else
	{
		if(area==1)
		{
			startPose.position.x = startPose.position.x+5;
			startPose.position.y = startPose.position.y+5;
		}
		else if(area==2)
		{
			startPose.position.x = startPose.position.x-5;
			startPose.position.y = startPose.position.y+5;

		}
		else if(area==3)
		{
			startPose.position.x = startPose.position.x+5;
			startPose.position.y = startPose.position.y-5;

		}
		else
		{
			startPose.position.x = startPose.position.x-5;
			startPose.position.y = startPose.position.y-5;
		}

	}

	//std::cout<<name<<": Pose x: "<<startPose.position.x<< "Pose y: "<<startPose.position.y<<std::endl;
	modelstate.pose = startPose;

	setmodelstate.request.model_state = modelstate;
}

visualization_msgs::Marker createMarker (std::string ns, int id, double x, double y,  double r = 1.0, double g = 0.0, double b = 0.0, double a = 1.0)
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
	printf("marker.pose.position.x %lf, marker.pose.position.y %lf: \n",marker.pose.position.x, marker.pose.position.y);

	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	return marker;
}

geometry_msgs::Pose globaleToRobotTransform(geometry_msgs::Pose pose)
{

	printf("pose.position.x: %lf, pose.position.y: %lf, pose.position.z: %lf \n",pose.position.x, pose.position.y, pose.position.z);
	printf("pose.orientation Yaw: %lf\n", tf::getYaw(pose.orientation) );
	// TF transformation of the Point which is nearest to the robot
	const ros::Time& scanTimeStamp = ros::Time (0);
	try
	{
		tfListener->waitForTransform (BASE_FRAME, MAP_FRAME, scanTimeStamp, ros::Duration (3.0));
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

	tf::Stamped<tf::Pose> transPose (tf::Pose( quat,origin), scanTimeStamp, MAP_FRAME);

	tf::Stamped<tf::Pose> poseTransformed;

	try
	{
		tfListener->transformPose(BASE_FRAME, scanTimeStamp, transPose, MAP_FRAME, poseTransformed);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR (" Point xyz %s", ex.what ());
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

	printf("pose.position.x: %lf, pose.position.y: %lf, pose.position.z: %lf \n",pose.position.x, pose.position.y, pose.position.z);
	printf("pose.orientation Yaw: %lf\n", tf::getYaw(pose.orientation) );

	return returnPose;
}
