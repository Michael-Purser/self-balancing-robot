#include "pid_controller/helpers.h"
#include "pid_controller/pid_controller.h"

#include <ros/console.h>
#include <ros/duration.h>

#include <std_msgs/Float64.h>

#include <iostream>
#include <string>

pid_controller::PIDController::PIDController(ros::NodeHandle& node_handle) :
    current_pitch_ {0.0}
{
  node_handle.getParam("/pid_controller/cutoff_pitch", cutoff_pitch_);
  node_handle.getParam("/pid_controller/pitch_setpoint", pitch_setpoint_);
  node_handle.getParam("/pid_controller/proportional_control_gain", proportional_control_gain_);
  node_handle.getParam("/pid_controller/spin_rate", spin_rate_);

  ros::Duration(1.0).sleep();

  std::string left_wheel_topic {"/teeterbot/left_speed_cmd"};
  std::string right_wheel_topic {"/teeterbot/right_speed_cmd"};

  imu_subscriber_ = node_handle.subscribe("/teeterbot/imu", 1, &PIDController::imuCallback, this);
  left_wheel_publisher_ = node_handle.advertise<std_msgs::Float64>("/teeterbot/left_speed_cmd", 1);
  right_wheel_publisher_ = node_handle.advertise<std_msgs::Float64>("/teeterbot/right_speed_cmd", 1);
}

void
pid_controller::PIDController::spin()
{
  ros::Rate spin_rate(spin_rate_);

  while(ros::ok())
  {
    ros::spinOnce();
    sendControlSignal();
    spin_rate.sleep();
  }
}

void
pid_controller::PIDController::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
  current_pitch_ = getPitchFromImuMessage(imu_msg);
}

void
pid_controller::PIDController::sendControlSignal()
{
  std_msgs::Float64 msg;
  if (-cutoff_pitch_ <= current_pitch_ && current_pitch_ <= cutoff_pitch_)
  {
    msg.data = proportional_control_gain_ * (current_pitch_ - pitch_setpoint_);
  }
  else
  {
    msg.data = 0.0;
  }
  left_wheel_publisher_.publish(msg);
  right_wheel_publisher_.publish(msg);
}
