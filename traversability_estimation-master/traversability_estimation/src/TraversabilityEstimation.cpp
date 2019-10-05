/*
 * TraversabilityEstimation.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "traversability_estimation/TraversabilityEstimation.hpp"
#include <traversability_msgs/TraversabilityResult.h>

// ROS
#include <ros/package.h>
#include <geometry_msgs/Pose.h>

using namespace std;

namespace traversability_estimation {

TraversabilityEstimation::TraversabilityEstimation(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      traversabilityMap_(nodeHandle),
      traversabilityType_("traversability"),
      slopeType_("traversability_slope"),
      stepType_("traversability_step"),
      roughnessType_("traversability_roughness"),
      robotSlopeType_("robot_slope"),
      getImageCallback_(false)
{
	std::cout<<"TraversabilityEstimation_ "<<__LINE__<<std::endl;

  ROS_DEBUG("Traversability estimation node started.");
  readParameters();
  submapClient_ = nodeHandle_.serviceClient<grid_map_msgs::GetGridMap>(submapServiceName_);

  if (!updateDuration_.isZero()) {
    updateTimer_ = nodeHandle_.createTimer(
        updateDuration_, &TraversabilityEstimation::updateTimerCallback, this);
  } else {
    ROS_WARN("Update rate is zero. No traversability map will be published.");
  }

  loadElevationMapService_ = nodeHandle_.advertiseService("load_elevation_map", &TraversabilityEstimation::loadElevationMap, this);
  updateTraversabilityService_ = nodeHandle_.advertiseService("update_traversability", &TraversabilityEstimation::updateServiceCallback, this);
  getTraversabilityService_ = nodeHandle_.advertiseService("get_traversability", &TraversabilityEstimation::getTraversabilityMap, this);
  footprintPathService_ = nodeHandle_.advertiseService("check_footprint_path", &TraversabilityEstimation::checkFootprintPath, this);
  updateParameters_ = nodeHandle_.advertiseService("update_parameters", &TraversabilityEstimation::updateParameter, this);
  traversabilityFootprint_ = nodeHandle_.advertiseService("traversability_footprint", &TraversabilityEstimation::traversabilityFootprint, this);

  saveToBagService_ = nodeHandle_.advertiseService("save_to_bag", &TraversabilityEstimation::saveToBag, this);
  //imageSubscriber_ = nodeHandle_.subscribe(imageTopic_,1,&TraversabilityEstimation::imageCallback, this);

 /* tf_prefix = ros::this_node::getNamespace();
  tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);
  std::size_t pos = tf_prefix.find("/");
  tf_prefix = tf_prefix.substr(0, pos-1);
*/
	tf_prefix = ros::this_node::getNamespace();
	if(tf_prefix == "/")
	{
		tf_prefix = "//GETjag1";
	}
	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	clcPathSrv = nodeHandle_.advertiseService("/"+tf_prefix+"/clcPath", &TraversabilityEstimation::clcPathCallback, this);

	std::istringstream iss (tf_prefix.substr(6, tf_prefix.size()));
	int robot_number = 1;
	iss >> robot_number;

  eleviationSubscriber_ = nodeHandle_.subscribe<grid_map_msgs::GridMap>("/"+tf_prefix+"/GridMap",1,boost::bind (&TraversabilityEstimation::elevationMapCallback, this, _1));
  GoalSubscriber = nodeHandle_.subscribe<nav_msgs::Odometry>("/"+tf_prefix+"/goal_pose",1,boost::bind(&TraversabilityEstimation::goalPoseCallback, this, _1));
  markerPublisher = nodeHandle_ .advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 20);
  clcThePath = false;
  goalSet = false;
  elevationMapLayers_.push_back("elevation");
  elevationMapLayers_.push_back("upper_bound");
  elevationMapLayers_.push_back("lower_bound");
  stetupMap = false;
  ROS_INFO("setupTraverabilityMap.\n");

}

TraversabilityEstimation::~TraversabilityEstimation()
{
  updateTimer_.stop();
  nodeHandle_.shutdown();
}

visualization_msgs::Marker TraversabilityEstimation::createMarker (std::string ns, int id, double x, double y,  double r, double g, double b, double a)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = tf_prefix+"/map";
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
	marker.pose.position.z = 0.2;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	return marker;
}

bool TraversabilityEstimation::clcPathCallback( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if(!stetupMap)
	{
		traversabilityMap_.setupTraverabilityMap(45);
		omplPlanner.setTraversabilityMap(&traversabilityMap_);

		stetupMap = true;
	}
	if(goalSet)
	{

		std::vector<pose> poses;
		traversabilityMap_.setcomputFootprint(true);

		omplPlanner.plan(goalPose,poses);

		traversabilityMap_.setcomputFootprint(false);

		visualization_msgs::MarkerArray markerArray;

		for(std::size_t i=0; i<poses.size();i++)
		{
			printf ("pose x: %4.4f pose y: %4.4f \n", poses.at(i).x, poses.at(i).y);
			markerArray.markers.push_back (createMarker("static line", i, poses.at(i).x, poses.at(i).y, 1.0, 1.0, 0.0, 1.0));
		}
		markerPublisher.publish(markerArray);
	}
	return true;
}


void TraversabilityEstimation::goalPoseCallback(const nav_msgs::Odometry::ConstPtr& goalPoseMsg)
{

	goalSet=true;
	goalPose.x =goalPoseMsg->pose.pose.position.x;
	goalPose.y =goalPoseMsg->pose.pose.position.y;
	goalPose.yaw = tf::getYaw(goalPoseMsg->pose.pose.orientation);

}
bool TraversabilityEstimation::readParameters()
{
  nodeHandle_.param("submap_service", submapServiceName_, string("get_grid_map"));
  //submapServiceName_ = tf_prefix + submapServiceName_;
  double updateRate;
  nodeHandle_.param("min_update_rate", updateRate, 1.0);
  if (updateRate != 0.0) {
    updateDuration_.fromSec(1.0 / updateRate);
  } else {
    updateDuration_.fromSec(0.0);
  }
  // Read parameters for image subscriber.
  nodeHandle_.param("image_topic", imageTopic_, std::string("/image_elevation"));
  nodeHandle_.param("resolution", imageResolution_, 0.06);
  nodeHandle_.param("min_height", imageMinHeight_, 0.0);
  nodeHandle_.param("max_height", imageMaxHeight_, 1.0);
  nodeHandle_.param("image_position_x", imagePosition_.x(), 0.0);
  nodeHandle_.param("image_position_y", imagePosition_.y(), 0.0);

  nodeHandle_.param("map_frame_id", mapFrameId_, string("map"));
  nodeHandle_.param("robot_frame_id", robotFrameId_, string("robot"));
  nodeHandle_.param("robot", robot_, string("robot"));
  nodeHandle_.param("package", package_, string("traversability_estimation"));
  grid_map::Position mapCenter;
  nodeHandle_.param("map_center_x", mapCenter.x(), 0.0);
  nodeHandle_.param("map_center_y", mapCenter.y(), 0.0);
  submapPoint_.header.frame_id = robotFrameId_;
  submapPoint_.point.x = mapCenter.x();
  submapPoint_.point.y = mapCenter.y();
  submapPoint_.point.z = 0.0;
  nodeHandle_.param("map_length_x", mapLength_.x(), 5.0);
  nodeHandle_.param("map_length_y", mapLength_.y(), 5.0);
  nodeHandle_.param("footprint_yaw", footprintYaw_, M_PI_2);

  nodeHandle_.param("elevation_map/topic", bagTopicName_,  std::string("grid_map"));
  nodeHandle_.param("elevation_map/load/path_to_bag", pathToLoadBag_, std::string("elevation_map.bag"));
  nodeHandle_.param("traversability_map/save/path_to_bag", pathToSaveBag_, std::string("traversability_map.bag"));

  //imageResolution_=0.06;
  return true;
}

bool TraversabilityEstimation::elevationMapCallback(const grid_map_msgs::GridMap::ConstPtr& gridMsg)
{
	sensor_msgs::Image img;
	grid_map::GridMap map;
	grid_map::GridMapRosConverter::fromMessage(*gridMsg,map);
	grid_map::GridMapRosConverter::toImage(map, "elevation", sensor_msgs::image_encodings::BGRA16 , img);

	if (!getImageCallback_)
	{
		imagePosition_= map.getPosition();
		grid_map::GridMapRosConverter::initializeFromImage(img, imageResolution_, imageGridMap_, imagePosition_);
		ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", imageGridMap_.getLength().x(), imageGridMap_.getLength().y(), imageGridMap_.getSize()(0), imageGridMap_.getSize()(1));
		imageGridMap_.add("upper_bound", 0.0); // TODO: Add value for layers.
		imageGridMap_.add("lower_bound", 0.0);
		imageGridMap_.add("uncertainty_range", imageGridMap_.get("upper_bound") - imageGridMap_.get("lower_bound"));
		getImageCallback_ = true;
		grid_map::GridMapRosConverter::addLayerFromImage(img, "elevation", imageGridMap_, imageMinHeight_, imageMaxHeight_);
		grid_map_msgs::GridMap elevationMap;
		grid_map::GridMapRosConverter::toMessage(imageGridMap_, elevationMap);
		traversabilityMap_.setElevationMap(elevationMap);
	}
}

bool TraversabilityEstimation::loadElevationMap(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("TraversabilityEstimation: loadElevationMap");
  grid_map::GridMap map;
  grid_map_msgs::GridMap msg;
  if (!grid_map::GridMapRosConverter::loadFromBag(pathToLoadBag_, bagTopicName_, map)) {
    ROS_ERROR("TraversabilityEstimation: Cannot find bag or topic of the elevation map!");
    return false;
  }
  for (auto layer : elevationMapLayers_) {
    if (!map.exists(layer)) {
      map.add(layer, 0.0);
      ROS_INFO_STREAM("TraversabilityEstimation: loadElevationMap: Added layer '" << layer << "'.");
    }
  }
  ROS_DEBUG_STREAM("Map frame id: " << map.getFrameId());
  for (auto layer : map.getLayers()) {
    ROS_DEBUG_STREAM("Map layers: " << layer);
  }
  ROS_DEBUG_STREAM("Map size: " << map.getLength());
  ROS_DEBUG_STREAM("Map position: " << map.getPosition());
  ROS_DEBUG_STREAM("Map resolution: " << map.getResolution());

  map.setTimestamp(ros::Time::now().toNSec());
  grid_map::GridMapRosConverter::toMessage(map, msg);
  traversabilityMap_.setElevationMap(msg);
  if (!traversabilityMap_.computeTraversability()) {
    ROS_WARN("TraversabilityEstimation: loadElevationMap: cannot compute traversability.");
    return false;
  }
  return true;
}

void TraversabilityEstimation::imageCallback(const sensor_msgs::Image& image)
{
  if (!getImageCallback_) {
    grid_map::GridMapRosConverter::initializeFromImage(image, imageResolution_, imageGridMap_, imagePosition_);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", imageGridMap_.getLength().x(), imageGridMap_.getLength().y(), imageGridMap_.getSize()(0), imageGridMap_.getSize()(1));
    imageGridMap_.add("upper_bound", 0.0); // TODO: Add value for layers.
    imageGridMap_.add("lower_bound", 0.0);
    getImageCallback_ = true;
  }
  grid_map::GridMapRosConverter::addLayerFromImage(image, "elevation", imageGridMap_, imageMinHeight_, imageMaxHeight_);
  grid_map_msgs::GridMap elevationMap;
  grid_map::GridMapRosConverter::toMessage(imageGridMap_, elevationMap);
  traversabilityMap_.setElevationMap(elevationMap);
}

void TraversabilityEstimation::updateTimerCallback(const ros::TimerEvent& timerEvent)
{
	updateTraversability();
}

bool TraversabilityEstimation::updateServiceCallback(grid_map_msgs::GetGridMapInfo::Request&, grid_map_msgs::GetGridMapInfo::Response& response)
{
  if (updateDuration_.isZero()) {
    if (!updateTraversability()) {
      ROS_ERROR("Traversability Estimation: Cannot update traversability!");
      return false;
    }
  }
  // Wait until traversability map is computed.
  while (!traversabilityMap_.traversabilityMapInitialized()) {
    sleep(1.0);
  }
  grid_map_msgs::GridMap msg;
  grid_map::GridMap traversabilityMap = traversabilityMap_.getTraversabilityMap();

  response.info.header.frame_id = mapFrameId_;
  response.info.header.stamp = ros::Time::now();
  response.info.resolution = traversabilityMap.getResolution();
  response.info.length_x = traversabilityMap.getLength()[0];
  response.info.length_y = traversabilityMap.getLength()[1];
  geometry_msgs::Pose pose;
  grid_map::Position position = traversabilityMap.getPosition();
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.orientation.w = 1.0;
  response.info.pose = pose;

  return true;
}

bool TraversabilityEstimation::updateTraversability()
{
  grid_map_msgs::GridMap elevationMap;
  if (!getImageCallback_) {
    ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
    ROS_INFO("Sending request to %s.", submapServiceName_.c_str());

    if (!submapClient_.waitForExistence(ros::Duration(2.0))) {
      return false;
    }
    ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
    ROS_INFO("Sending request to %s.", submapServiceName_.c_str());

    if (requestElevationMap(elevationMap)) {
      traversabilityMap_.setElevationMap(elevationMap);
      if (!traversabilityMap_.computeTraversability()) return false;
    } else {
      ROS_WARN("Failed to retrieve elevation grid map.");
      return false;
    }
  } else {
    if (!traversabilityMap_.computeTraversability()) return false;
  }

  return true;
}

bool TraversabilityEstimation::updateParameter(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Load parameters file.
  string path = ros::package::getPath(package_);
  string path_filter_parameter = path + "/config/" + robot_ + "_filter_parameter.yaml";
  string path_footprint_parameter = path + "/config/" + robot_ + "_footprint_parameter.yaml";
  // Filter parameters
  string commandString = "rosparam load " + path_filter_parameter + " /traversability_estimation";
  const char* command_filter = commandString.c_str();
  if (system(command_filter) != 0)
  {
    ROS_ERROR("Can't update parameter.");
    return false;
  }
  // Footprint parameters.
  commandString = "rosparam load " + path_footprint_parameter + " /traversability_estimation";
  const char* command_footprint = commandString.c_str();
  if (system(command_footprint) != 0)
  {
    ROS_ERROR("Can't update parameter.");
    return false;
  }

  if (!traversabilityMap_.updateFilter()) return false;
  return true;
}

bool TraversabilityEstimation::requestElevationMap(grid_map_msgs::GridMap& map)
{
  submapPoint_.header.stamp = ros::Time(0);
  geometry_msgs::PointStamped submapPointTransformed;

  try {
    transformListener_.transformPoint(mapFrameId_, submapPoint_,
                                      submapPointTransformed);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  grid_map_msgs::GetGridMap submapService;
  submapService.request.position_x = submapPointTransformed.point.x;
  submapService.request.position_y = submapPointTransformed.point.y;
  submapService.request.length_x = mapLength_.x();
  submapService.request.length_y = mapLength_.y();
  submapService.request.layers = elevationMapLayers_;

  if (!submapClient_.call(submapService))
    return false;
  map = submapService.response.map;

  return true;
}

bool TraversabilityEstimation::traversabilityFootprint(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
//traversabilityMap_.setupTraverabilityMap(45);

	stetupMap = true;

	std::vector<pose> poses;
	traversabilityMap_.setcomputFootprint(true);


	if (!traversabilityMap_.traversabilityFootprint(45))
		return false;

	omplPlanner.setTraversabilityMap(&traversabilityMap_);
	omplPlanner.plan(goalPose,poses);

	visualization_msgs::MarkerArray markerArray;

	for(std::size_t i=0; i<poses.size();i++)
	{
		printf ("pose x: %4.4f pose y: %4.4f \n", poses.at(i).x, poses.at(i).y);
		markerArray.markers.push_back (createMarker("static line", i, poses.at(i).x, poses.at(i).y, 1.0, 1.0, 0.0, 1.0));
	}
	markerPublisher.publish(markerArray);
	traversabilityMap_.setcomputFootprint(false);

  return true;
}

bool TraversabilityEstimation::checkFootprintPath(
    traversability_msgs::CheckFootprintPath::Request& request,
    traversability_msgs::CheckFootprintPath::Response& response)
{
  const int nPaths = request.path.size();
  if (nPaths == 0) {
    ROS_WARN("No footprint path available to check!");
    return false;
  }

  traversability_msgs::TraversabilityResult result;
  traversability_msgs::FootprintPath path;
  for (int j = 0; j < nPaths; j++) {
    path = request.path[j];
    if (!traversabilityMap_.checkFootprintPath(path, result, true))
      return false;
    response.result.push_back(result);
  }

  return true;
}

bool TraversabilityEstimation::getTraversabilityMap(
    grid_map_msgs::GetGridMap::Request& request,
    grid_map_msgs::GetGridMap::Response& response)
{
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
  grid_map_msgs::GridMap msg;
  grid_map::GridMap map, subMap;
  map = traversabilityMap_.getTraversabilityMap();
  bool isSuccess;
  subMap = map.getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
  if (request.layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    vector<string> layers;
    for (const auto& layer : request.layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response.map);
  }
  return isSuccess;
}

bool TraversabilityEstimation::saveToBag(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Save to bag.");
  return grid_map::GridMapRosConverter::saveToBag(traversabilityMap_.getTraversabilityMap(), pathToSaveBag_, bagTopicName_);
}

} /* namespace */
