#include <teeterbot_listener/teeterbot_listener.h>

#include <ros/console.h>
#include <ros/rate.h>

teeterbot_listener::TeeterbotListener::TeeterbotListener() :
    joint_state_topic_name_{"/teeterbot/joint_states"}
{
  ROS_INFO_STREAM("Subscribing to topic: " << joint_state_topic_name_);
  ros::NodeHandle node_handle;
  joint_state_sub_ = node_handle.subscribe(joint_state_topic_name_, 1, &TeeterbotListener::jointStateCallback, this);

  ROS_INFO_STREAM("Spinning...");
  spin();
}

void
teeterbot_listener::TeeterbotListener::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  ROS_INFO_STREAM("Message Sequence no: " << std::to_string(msg->header.seq));
  ROS_INFO_STREAM("    Left wheel position: " << std::to_string(msg->position.front()));
  ROS_INFO_STREAM("    Right wheel position: " << std::to_string(msg->position.back()));
}

void
teeterbot_listener::TeeterbotListener::spin() const
{
  ros::Rate rate(100.0);

  while (ros::ok())
  {
    if (joint_state_sub_.getNumPublishers() > 0)
    {
      ros::spinOnce();
    }
    else
    {
      ROS_WARN_STREAM("Topic " << joint_state_topic_name_ << " has no publishers!");
    }
    rate.sleep();
  }
}
