#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "elevationmapper.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/crop_box.h>
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/Empty.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <ros/package.h>

tf::TransformListener* listener=NULL;
ElevationMapper* mapper=NULL;
ros::Publisher publisher;
ros::Publisher elevationMapImagePublisher;
double eleviation_mapping_resulution = 0.06;
double creatMapszieX = 105;
double creatMapszieY = 105;
bool resetMap = false;

std::string BASE_FRAME = "/base_link";
std::string MAP_FRAME = "/map";
std::string gazeboMoveObjectFrame = "world";
std::string tf_prefix = "GETjag";
ros::NodeHandle* nodeHandel;

pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in, bool filterBox = true) {
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelGridFilter;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boxFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxelFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> indices;
	tf::StampedTransform tempTf;
	Eigen::Affine3d tempTfEigen;
	//std::cout <<"cloud size " << cloud_in->size() << std::endl;

	if (filterBox) {
		if(!listener->waitForTransform(BASE_FRAME, cloud_in->header.frame_id, pcl_conversions::fromPCL(cloud_in->header.stamp), ros::Duration(0.2))) {
			std::cout << "Scan Transform Timeout before BoxFilter" << std::endl;
			throw std::exception();
		}
		listener->lookupTransform(MAP_FRAME, cloud_in->header.frame_id, pcl_conversions::fromPCL(cloud_in->header.stamp), tempTf);
		tf::transformTFToEigen(tempTf, tempTfEigen);

		pcl::CropBox<pcl::PointXYZ> boxFilter;
		boxFilter.setInputCloud(cloud_in);
		boxFilter.setMin(Eigen::Vector4f(-0.5, -0.4, -0.5, 1.0));
		boxFilter.setMax(Eigen::Vector4f(0.55, 0.4, 0.5, 1.0));
		boxFilter.setTransform(tempTfEigen.cast<float>());
		boxFilter.setNegative(true);
		boxFilter.filter(*boxFilteredCloud);

		voxelGridFilter.setInputCloud(boxFilteredCloud);
	}
	else
		voxelGridFilter.setInputCloud(cloud_in);

	voxelGridFilter.setDownsampleAllData(false);
	voxelGridFilter.setLeafSize(0.9 * eleviation_mapping_resulution, 0.9 * eleviation_mapping_resulution, 0.9 * eleviation_mapping_resulution);
	voxelGridFilter.filter(*voxelFilteredCloud);


	pcl::removeNaNFromPointCloud(*voxelFilteredCloud, *voxelFilteredCloud, indices);
    //std::cout <<"cloud size after " << voxelFilteredCloud->size() << std::endl;

	return voxelFilteredCloud;
}

std::string cheakRectBound(const cv::Mat& image, const cv::Rect& myROI)
{
	std::string error;
	if (0 <= myROI.x)
	{
		error = "0 <= Rect.x";
	}
	else if(0 <= myROI.width)
	{
		error = "0 <= Rect.width";
	}
	else if(myROI.x + myROI.width <= image.cols)
	{
		error = "Rect.x + Rect.width <= image.width";
	}
	else if(0 <= myROI.y)
	{
		error = "0 <= Rect.y";
	}
	else if(0 <= myROI.height)
	{
		error = "0 <= Rect.height";
	}
	else
	{
		error = "Rect.y + Rect.height <= cvImage.height";
	}

	error.clear();
	return error;
}

void getCropedImage(cv_bridge::CvImage& cv_ptr, int widthX , int widthY)
{
	cv::Mat cvImage = cv_ptr.image;
	//cvtColor(cvImage, cvImage, cv::COLOR_BGRA2GRAY);
//cv_ptr.encoding
	int x = cvImage.cols/2 - widthX/2;
	int y = cvImage.rows/2 - widthY/2;
	cv::Rect myROI(x, y , widthX, widthY);

	std::string rectError = cheakRectBound(cvImage,myROI);

	if (rectError.empty())
	{
		cvImage = cvImage(myROI);
	}
	else
	{
		printf("this is not working because: %s\n",rectError.c_str());
	}
	cv_ptr.image = cvImage;
}

void msgCallback(const ros::TimerEvent&)
{
	publisher.publish(*mapper->getMapMessage());

	if(nodeHandel->hasParam("reset_elevation_map"))
	{
		nodeHandel->getParam("reset_elevation_map",resetMap);

	}
	else
	{
		nodeHandel->setParam("reset_elevation_map",false);
		resetMap=true;
	}
	if(resetMap==true)
	{
		mapper->resetMap(creatMapszieX, creatMapszieY, eleviation_mapping_resulution, MAP_FRAME);
		resetMap=false;
		nodeHandel->setParam("reset_elevation_map",false);
	}
	cv_bridge::CvImage cv_ptr;
	grid_map::GridMap map = mapper->getMap();
	//grid_map::GridMapRosConverter::toCvImage(map, std::string("elevation") ,  "8UC1" , cv_ptr);
	grid_map::GridMapRosConverter::toCvImage(map, std::string("elevation") ,  "16UC1" , cv_ptr);

	
	Eigen::Array2i mapSize = map.getSize();
	
	int widthX = 1000;
	int widthY = 1000;
	if(mapSize[0]>widthX && mapSize[1]> widthY)
	{
		getCropedImage(cv_ptr, widthX, widthY);
	}
	
	cv::Mat image = cv_ptr.image;
	//cv_ptr.
	//cv::imshow("hklhsadh",image);
	//cv::waitKey(1);
	//cv::imwrite((ros::package::getPath("elevation_mapper")+"/image.jpg").c_str(),image);
	sensor_msgs::Image pubImage;
	cv_ptr.toImageMsg(pubImage);
	elevationMapImagePublisher.publish(pubImage);

}

void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in) {
	try {
		pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = preprocessCloud(cloud_in);
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl_ros::transformPointCloud(MAP_FRAME, *filteredCloud, *transformedCloud, *listener);
		mapper->addPointCloud(transformedCloud);
	}
	catch (const std::exception& e) {
		return;
	}
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "elevation_mapper_node");
	ros::NodeHandle nh;
	nodeHandel = &nh;
	tf_prefix = ros::this_node::getNamespace();
	if(tf_prefix == "/")
	{
		tf_prefix = "//GETjag1";
	}
	tf_prefix = tf_prefix.substr(2, tf_prefix.size()-1);

	BASE_FRAME = tf_prefix + BASE_FRAME;
	MAP_FRAME = tf_prefix + MAP_FRAME;
	nh.setParam("reset_elevation_map",false);

	publisher = nh.advertise<grid_map_msgs::GridMap>("GridMap", 10);
	elevationMapImagePublisher = nh.advertise<sensor_msgs::Image>("elevation_map_image", 1);

	ElevationMapper temp(creatMapszieX, creatMapszieY, eleviation_mapping_resulution, MAP_FRAME);
	mapper = &temp;
    listener = new(tf::TransformListener);

	ros::Subscriber CloudListener = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("xtion/depth/points", 10, cloudCallback);
	ros::Timer msg_timer = nh.createTimer(ros::Duration(1), msgCallback);

	ros::spin();


}
