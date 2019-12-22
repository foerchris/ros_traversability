/*
 * FlipperControl.h
 *
 *  Created on: 12.12.2018
 *      Author: chfo
 */

#ifndef ROS_ROBOCUP_FLIPPERCONTROL_SRC_CropMap_H_
#define ROS_ROBOCUP_FLIPPERCONTROL_SRC_CropMap_H_

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <nav_msgs/Odometry.h>

// Tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// marker array
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

struct Vector3
{
    float x, y, z;
};
class CropMap
{
	public:
	/*!
	 * Constructor.
	 * @param nodeHandle the ROS node handle.
	 */
	CropMap();

	/*!
	 * Destructor.
	 */
	virtual ~CropMap();

	/*!
	 * returns the a region according to the  width and length from the map given its coordinate frame
	 * @param mapImage; image of the map
	 * @param regionLength; size of cut out section
	 * @param regionWidth; size of cut out section
	 * @param destination_frame; frame of the map
	 * @param original_frame; frame of the cut out section
	 * @return return poses
	 */
	std::vector<geometry_msgs::Pose> getRegions(cv::Mat mapImage,const double& regionLength, const double& regionWidth, const std::string& destination_frame, const std::string& original_frame);

	/*!
	 * returns the a region according to the  width and length from the map given its coordinate frame
	 * @param mapImage; image of the map
	 * @param regionLength; size of cut out section
	 * @param regionWidth; size of cut out section
	 * @param destination_frame; frame of the map
	 * @param original_frame; frame of the cut out section
	 * @return croped image
	 */
	cv::Mat getRobotGroundImage(cv::Mat mapImage, const double& regionLength, const double& regionWidth, const std::string& destination_frame, const std::string& original_frame);


	/*!
	 * cropes the image with an rotated rect according to the position and orientation of the robot inside the map
	 * @param pose; current robots position
	 * @param mapImage; image of the map
	 * @param rectSizeX; size in x coordinates
	 * @param rectSizeY; size in y coordinates
	 * @return
	 */
	cv::Mat getCropedImage(geometry_msgs::Pose& pose, cv::Mat mapImage,double rectSizeX, double rectSizeY);

	/*!
	 * get poses from the image
	 * @param flipperMaps; image
	 * @param nextPose; next poses estimated by the linear and angular velocity
	 * @param destination_frame; frame of the map
	 * @param original_frame; frame of the cut out section
	 * @return
	 */
	std::vector<geometry_msgs::Pose> getPosesFromImage(cv::Mat flipperMaps, const geometry_msgs::Pose& nextPose, const std::string& destination_frame,const std::string& original_frame);

	/*!
	 * calculate the max z value elevation map section
	 * @param poses;
	 * @param q; quaternion of the hyperplane
	 * @param trackLength; length of the tracks
	 * @return max z value
	 */
	double clcMaxZ(const std::vector<geometry_msgs::Pose>& poses, tf2::Quaternion q, const double& trackLength);

	/*!
	 * calculate flipper positon
	 * @param poses
	 * @param q; quaternion of the hyperplane
	 * @param maxZ; max z value
	 * @param baseFrame; center of robot
	 * @param flipperFrame; frame of the flipper
	 * @param flipperRegionFrame; center frame of the flipper
	 * @return flipper position
	 */
	std::vector<geometry_msgs::Pose> clcNewFlipperPoses(const std::vector<geometry_msgs::Pose>& poses, tf2::Quaternion q, const double maxZ, const std::string& baseFrame, const std::string& flipperFrame, const std::string& flipperRegionFrame);


	//****************************************** helper functions
	/*!
	 * set the image resulution
	 * @param imageResultion; resulution of image
	 */
	void setConstants(const double& imageResultion);


	/*!
	 * transform vector of poses
	 * @param poses; vector of poses
	 * @param destination_frame;
	 * @param original_frame;
	 * @return transformed vector of poses
	 */
	std::vector<geometry_msgs::Pose> transformPose(const std::vector<geometry_msgs::Pose>& poses,const std::string& destination_frame,const std::string& original_frame);

	/*!
	 * tf transformation method
	 * @param pose
	 * @param destination_frame
	 * @param original_frame
	 * @return transformed pose
	 */
	geometry_msgs::Pose tfTransform(const geometry_msgs::Pose& pose,const std::string& destination_frame,const std::string& original_frame);

	/*!
	 * draws the rotated rectangle for the flipper region into the image for debuging
	 * @param image
	 * @param rotatedRectangle
	 */
	void DrawRotatedRectangle(cv::Mat& image, cv::RotatedRect rotatedRectangle);


	/*!
	 * tranform quaternion
	 * @param q; original quaternion
	 * @param destination_frame
	 * @param original_frame
	 * @param setRoll; set roll angle to 0
	 * @param setPitch; set pitch angle to 0
	 * @return destination quaternion
	 */
	tf2::Quaternion getDestQuat(tf2::Quaternion q, const std::string& destination_frame, const std::string& original_frame, const double& setRoll, const double& setPitch);

	// display the each point through a marker array
	visualization_msgs::MarkerArray creatMarkerArrayFlipperLine(const std::vector<geometry_msgs::Pose>& pose, const std::string& name, const std::string& frame, float r, float g, float b);
	visualization_msgs::MarkerArray creatMarkerArrayFlipperPoints(const std::vector<geometry_msgs::Pose>& pose, const std::string& name, const std::string& frame, float r, float g, float b);
	visualization_msgs::Marker createMarker (const std::string& tfFrame, const std::string& ns,const int& id,const double& x,const double& y,const double& z,const  double& r,const double& g,const double& b,const double& a);

	visualization_msgs::MarkerArray creatCubeMarkerArrayFlipperPoints(const geometry_msgs::Pose& pose, const std::string& name, const std::string& frame, float r, float g, float b);
	visualization_msgs::Marker createCubeMarker (const std::string& tfFrame, const std::string& ns,const int& id,const geometry_msgs::Pose& pose,const  double& r,const double& g,const double& b,const double& a);

	private:
	// *********** definitions for tf tranform
	/*!
	 * calculate distant between 2 points
	 * @param pose1; point 1
	 * @param pose2; point 2
	 * @return distant
	 */
	double clcDistanz(const geometry_msgs::Pose& pose1,const geometry_msgs::Pose& pose2);

	/*!
	 * rotated point by theta
	 * @param pPose
	 * @param theta
	 * @param oPose
	 * @return rotated point
	 */
	geometry_msgs::Pose rotate_point(geometry_msgs::Pose pPose ,const float& theta,const geometry_msgs::Pose& oPose);

	/*!
	 * get croped image with datatype 16US1
	 * @param pose; pose of where to cut out section
	 * @param mapImage; elevation map image
	 * @param rectSizeX;
	 * @param rectSizeY
	 * @return
	 */
	cv::Mat getCropedImage16US1(geometry_msgs::Pose& pose, cv::Mat mapImage, double rectSizeX, double rectSizeY);

	/*!
	 * transform poses by the hyperplane
	 * @param poses
	 * @param q
	 * @return transformed poses
	 */
	std::vector<geometry_msgs::Pose> clcNewPoses(const std::vector<geometry_msgs::Pose>& poses, tf2::Quaternion q);


	/*!
	 * transform pose by hyperplane
	 * @param pose
	 * @param q
	 * @return transformed poses
	 */
	geometry_msgs::Pose clcQuternion(const geometry_msgs::Pose& pose,const tf2::Quaternion& q);


	visualization_msgs::Marker createLineMarker (const std::string& tfFrame, const std::string& ns,const int& id, const std::vector<geometry_msgs::Pose>& poses,const  double& r,const double& g,const double& b,const double& a);

	std::unique_ptr<tf::TransformListener> tfListener;
	tf::StampedTransform transform;

	std::string tf_prefix;

	//*****
	double resultion;
	double mapSizeX;
	double mapSizeY;

};


#endif /* ROS_ROBOCUP_FLIPPERCONTROL_SRC_CropMap_H_ */
