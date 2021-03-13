#ifndef PID_CONTROLLER_PID_CONTROLLER_ROS_INTERFACE
#define PID_CONTROLLER_PID_CONTROLLER_ROS_INTERFACE

#include "pid_controller/pid_controller.h"

#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <sensor_msgs/Imu.h>

#include <string>

namespace pid_controller
{

class PIDControllerROSInterface
{
public:
  PIDControllerROSInterface(ros::NodeHandle& node_handle);

  void spin();

private:
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  void printInterfaceParams() const;
  void printControllerParams() const;
  void publishTopics();

  PIDController controller_;
  double spin_rate_;

  ros::Subscriber imu_subscriber_;
  ros::Publisher controller_info_publisher_;
  ros::Publisher left_wheel_speed_cmd_publisher_;
  ros::Publisher right_wheel_speed_cmd_publisher_;
};

std::string controllerStatusToMsgStatus(const PIDControllerStatus status);

}

#endif
