#ifndef PID_CONTROLLER_HELPERS
#define PID_CONTROLLER_HELPERS

#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

namespace pid_controller
{

double getPitchFromImuMessage(const sensor_msgs::ImuConstPtr& imu_msg);

}

#endif
