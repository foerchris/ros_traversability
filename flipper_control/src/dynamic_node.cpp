#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <flipper_control/broadcasterConfig.h>

void callback(flipper_control::broadcasterConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
            config.int_param, config.fixed_velocity,
            config.str_param.c_str(),
            config.bool_param?"True":"False",
            config.size);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_tutorials");

  dynamic_reconfigure::Server<flipper_control::broadcasterConfig> server;
  dynamic_reconfigure::Server<flipper_control::broadcasterConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
