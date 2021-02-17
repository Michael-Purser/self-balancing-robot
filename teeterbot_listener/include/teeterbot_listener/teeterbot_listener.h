#ifndef TEETERBOT_LISTENER_TEETERBOT_LISTENER
#define TEETERBOT_LISTENER_TEETERBOT_LISTENER

#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

#include <string>

namespace teeterbot_listener
{

class TeeterbotListener
{
public:
  TeeterbotListener();

private:
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  void spin() const;

  std::string joint_state_topic_name_;
  ros::Subscriber joint_state_sub_;
};

}

#endif
