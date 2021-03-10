#include "pid_controller/pid_controller.h"

#include <ros/init.h>

#include <iostream>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_controller");

  ros::NodeHandle node_handle;

  pid_controller::PIDController controller(node_handle);

  controller.spin();

  return 0;
}
