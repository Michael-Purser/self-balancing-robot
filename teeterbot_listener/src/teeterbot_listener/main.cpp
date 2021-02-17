#include "teeterbot_listener/teeterbot_listener.h"

#include <ros/init.h>
#include <ros/node_handle.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "teeterbot_listener");

  teeterbot_listener::TeeterbotListener listener;
}
