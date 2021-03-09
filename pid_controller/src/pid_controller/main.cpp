#include <ros/init.h>

#include <iostream>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_controller");

  std::cout << "Hello" << std::endl;

  return 0;
}
