#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;

  ros::Rate r(30);
  float env_size = 10.0;
  float f = 0.0;
//while (ros::ok())
 //{
  std::vector <float> xes;
	  for (float x = env_size/8; x <= env_size - env_size/8; x += env_size/4)
	  {
		  for (float y = env_size/8; y <= env_size - env_size/8; y += env_size/4)
		  {
			  xes.push_back(x);
		  }
	  }
	  std::cout<<"xes.size()"<< xes.size() << std::endl;

	  std::vector<int> forbidden_list;
//forbidden_list.push_back(1);
	  forbidden_list.push_back(2);
	  forbidden_list.push_back(3);
	  forbidden_list.push_back(4);

	  if(std::find(forbidden_list.begin(), forbidden_list.end(), 1) != forbidden_list.end())
	  {
	      printf(" v contains x ");
	  }
    r.sleep();
  //}
}
