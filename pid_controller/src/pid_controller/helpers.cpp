#include "pid_controller/helpers.h"

double
pid_controller::getPitchFromImuMessage(const sensor_msgs::ImuConstPtr& imu_msg)
{
  tf::Quaternion quaternion(
    imu_msg->orientation.x,
    imu_msg->orientation.y,
    imu_msg->orientation.z,
    imu_msg->orientation.w);
  tf::Matrix3x3 matrix(quaternion);
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);

  return pitch;
}
