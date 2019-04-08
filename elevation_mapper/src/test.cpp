#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;

  ros::Rate r(1);
  float env_size = 10.0;
  float f = 0.0;
  while (ros::ok())
  {
	  int zahl1=0, zahl2=3;
	  std::vector<int> bla;


	  for (zahl1; zahl1<zahl2; zahl1++)
	  {
		  bla.push_back(zahl1);

		  printf("zahl1 = %i\n", zahl1);
		  printf("zahl2 = %i\n", zahl2);

	  }

	  for (auto blabla: bla)
	  {
		  printf("blabla = %i\n", blabla);

	  }
      r.sleep();
  }
}
