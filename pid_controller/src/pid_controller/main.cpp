#include "pid_controller/pid_controller_ros_interface.h"

#include <ros/init.h>

#include <iostream>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_controller");

  ros::NodeHandle node_handle;

  pid_controller::PIDControllerROSInterface controller_interface(node_handle);

  controller_interface.spin();

  return 0;
}
