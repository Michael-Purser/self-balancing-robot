#include "pid_controller/helpers.h"
#include "pid_controller/pid_controller_ros_interface.h"

#include <ros/console.h>
#include <ros/duration.h>

#include <std_msgs/Float64.h>

#include <iostream>
#include <string>

pid_controller::PIDControllerROSInterface::PIDControllerROSInterface(ros::NodeHandle& node_handle)
{
  node_handle.getParam("/pid_controller/cutoff_pitch", controller_.cutoff_value);
  node_handle.getParam("/pid_controller/pitch_setpoint", controller_.setpoint);
  node_handle.getParam("/pid_controller/proportional_control_gain", controller_.proportional_control_gain);
  node_handle.getParam("/pid_controller/integral_control_gain", controller_.integral_control_gain);
  node_handle.getParam("/pid_controller/derivative_control_gain", controller_.derivative_control_gain);
  node_handle.getParam("/pid_controller/spin_rate", spin_rate_);

  printInterfaceParams();
  printControllerParams();
  ros::Duration(1.0).sleep();

  imu_subscriber_ = node_handle.subscribe("/teeterbot/imu", 1, &PIDControllerROSInterface::imuCallback, this);
  left_wheel_speed_cmd_publisher_ = node_handle.advertise<std_msgs::Float64>("/teeterbot/left_speed_cmd", 1);
  right_wheel_speed_cmd_publisher_ = node_handle.advertise<std_msgs::Float64>("/teeterbot/right_speed_cmd", 1);
}

void
pid_controller::PIDControllerROSInterface::spin()
{
  ROS_INFO_STREAM("Starting main control loop");

  ros::Rate spin_rate(spin_rate_);

  while(ros::ok())
  {
    ros::spinOnce();
    publishTopics();
    spin_rate.sleep();
  }
}

void
pid_controller::PIDControllerROSInterface::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
  controller_.current_value = getPitchFromImuMessage(imu_msg);
}

void
pid_controller::PIDControllerROSInterface::printInterfaceParams() const
{
  ROS_INFO_STREAM("INTERFACE PARAMETER: spin rate: " << spin_rate_);
}

void
pid_controller::PIDControllerROSInterface::printControllerParams() const
{
  ROS_INFO_STREAM("CONTROLLER PARAMETER: cutoff value: " << controller_.cutoff_value);
  ROS_INFO_STREAM("CONTROLLER PARAMETER: setpoint: " << controller_.setpoint);
  ROS_INFO_STREAM("CONTROLLER PARAMETER: P gain: " << controller_.proportional_control_gain);
  ROS_INFO_STREAM("CONTROLLER PARAMETER: I gain: " << controller_.integral_control_gain);
  ROS_INFO_STREAM("CONTROLLER PARAMETER: D gain: " << controller_.derivative_control_gain);
}

void
pid_controller::PIDControllerROSInterface::publishTopics()
{
  std_msgs::Float64 msg;
  msg.data = controller_.computeNextControlSignal();
  left_wheel_speed_cmd_publisher_.publish(msg);
  right_wheel_speed_cmd_publisher_.publish(msg);
}
