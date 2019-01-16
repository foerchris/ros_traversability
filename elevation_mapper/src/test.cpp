#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>

using namespace cv;


void rotCropImg()
{
	std::string bla = ros::package::getPath("elevation_mapper")+"/image/Download.jpeg";
    std::cout<<bla<<std::endl;

	cv::Mat image = imread(bla,CV_LOAD_IMAGE_COLOR);
    std::cout<<"input image size: x="<<image.cols<<", y="<<image.rows<<std::endl;

    imshow("input",image);


	cv::Point2f point(100, 100);
	cv::Size2f size(50,50);

	// rect is the RotatedRect
	cv::RotatedRect rect = cv::RotatedRect(point, size,180);
    // matrices we'll use
    Mat M, rotated, cropped;
    // get angle and size from the bounding box
    float angle = rect.angle;
    Size rect_size = rect.size;
    // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
    if (rect.angle < -45.) {
        angle += 90.0;
        swap(rect_size.width, rect_size.height);
    }
    // get the rotation matrix
    M = getRotationMatrix2D(rect.center, angle, 1.0);
    // perform the affine transformation
    warpAffine(image, rotated, M, image.size(), 2);
    // crop the resulting image
    getRectSubPix(rotated, rect_size, rect.center, cropped);
	std::cout<<"cropped size: x="<<cropped.cols<<", y="<<cropped.rows<<std::endl;

    imshow("output",cropped);
	waitKey(1);
}

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
	  std::vector<int> v{ 21, 20, -14, 1, 5, 9 };
	     std::vector<int>::iterator result;

	     result = std::max_element(v.begin(), v.end());
	     std::cout << "max element at: " << std::distance(v.begin(), result) << '\n';

	     result = std::max_element(v.begin(), v.end(), abs_compare);
	     std::cout << "max element (absolute) at: " << std::distance(v.begin(), result);
	  ros::spinOnce();
  }
  return 0;
}
