// Copyright [2015]
#ifndef PANDUBOT_PUBLISHERS_HPP
#define PANDUBOT_PUBLISHERS_HPP

#include <ros/ros.h>
#include <string>
#include "pandubot_msgs/MuxMessage.h"
#include "pandubot_msgs/MuxControl.h"
#include <move_base_msgs/MoveBaseAction.h>

namespace pandubot_publishers {
class MoveBaseMuxControl {
 private:
  ros::NodeHandle& nh_;
  ros::Publisher pub_;
 public:
  MoveBaseMuxControl(ros::NodeHandle& nh, std::string topic_name);
  void Publish(std::string sender_id, std::string switch_to_id);
};

class MoveBaseMuxMessage {
 private:
  ros::NodeHandle &nh_;
  ros::Publisher pub_;
 public:
  MoveBaseMuxMessage(ros::NodeHandle& nh, std::string topic_name);
  void Publish(std::string sender_id, move_base_msgs::MoveBaseGoal goal);
};
}  // namespace pandubot_publishers

#endif // PANDUBOT_PUBLISHERS_HPP