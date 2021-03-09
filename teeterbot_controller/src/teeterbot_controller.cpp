#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>

#include <cmath>
#include <iostream>
#include <stdlib.h>

double robot_pitch_setpoint_ = 0.0;
double robot_pitch_;
double pid_proportional_gain_ = 1.0;

void
imuSubscriberCallback(const sensor_msgs::ImuConstPtr msg)
{
  tf::Quaternion quaternion(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
  tf::Matrix3x3 matrix(quaternion);
  double roll, yaw;
  matrix.getRPY(roll, robot_pitch_, yaw);
  // ROS_INFO_STREAM("Orientation pitch: " << robot_pitch_);
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "teeterbot_controller");
  ros::NodeHandle nh {};

  ros::Subscriber imu_subscriber {nh.subscribe("/teeterbot/imu", 1, imuSubscriberCallback)};

  ros::Publisher left_wheel_publisher {nh.advertise<std_msgs::Float64>("/teeterbot/left_speed_cmd", 1)};
  ros::Publisher right_wheel_publisher {nh.advertise<std_msgs::Float64>("/teeterbot/right_speed_cmd", 1)};

  nh.getParam("/teeterbot_controller/proportional_gain", pid_proportional_gain_);
  ROS_INFO_STREAM("Controller Proportional Gain: " << pid_proportional_gain_);

  ros::Duration(2.0).sleep();

  ros::Rate rate(100.0);

  std_msgs::Float64 msg;

  while (ros::ok())
  {
    ros::spinOnce();

    double error = robot_pitch_ - robot_pitch_setpoint_;

    if (abs(robot_pitch_) < 0.3)
    {
      double sign = signbit(error) ? -1.0 : 1.0;
      msg.data = sign * pid_proportional_gain_ * std::pow(abs(error), 0.5);
      ROS_INFO_STREAM("Published control signal: " << msg.data);
    }
    else
    {
      msg.data = 0.0;
    }
    left_wheel_publisher.publish(msg);
    right_wheel_publisher.publish(msg);

    rate.sleep();
  }

  return 0;
}
