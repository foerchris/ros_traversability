#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>

using namespace cv;



static bool abs_compare(int a, int b)
{
    return (std::abs(a) < std::abs(b));
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  while(ros::ok())
  {
	  std::vector<double> v{ 3.2, 1.4, -14, 1, 5, 9 };
	  auto result = std::max_element(v.begin(), v.end());

	  std::cout << "max element: " << *result << '\n';


	  ros::spinOnce();
  }
  return 0;
}
