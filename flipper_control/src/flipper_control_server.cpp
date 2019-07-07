/*
 * flipper_control_server.cpp
 *
 *  Created on: 12.12.2018
 *      Author: chfo
 */

#include <flipper_control/FlipperControl.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "flipper_control_server");
	ros::NodeHandle nodeHandel;
	FlipperControl flipperControl(nodeHandel);
	ros::spin();
	return 0;
}
