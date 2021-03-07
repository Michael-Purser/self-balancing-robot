#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/subscriber.h>

#include <sensor_msgs/Imu.h>

#include <iostream>

void
imuSubscriberCallback(const sensor_msgs::Imu& msg)
{
  ROS_INFO_STREAM("Orientation x: " << msg.orientation.x);
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "teeterbot_controller");
  ros::NodeHandle nh {};

  ros::Subscriber imu_subscriber {nh.subscribe("/teeterbot/imu", 1, imuSubscriberCallback)};

  ros::Rate rate(100.0);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}