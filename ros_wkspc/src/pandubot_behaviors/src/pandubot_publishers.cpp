#include "pandubot_publishers.hpp"

using std::string;

namespace pandubot_publishers {
MoveBaseMuxControl::MoveBaseMuxControl(ros::NodeHandle& nh, string topic_name)
: nh_(nh) {
  pub_ = nh_.advertise<pandubot_msgs::MuxControl>(topic_name,5);
}

void MoveBaseMuxControl::Publish(string sender_id, string switch_to_id) {
  pandubot_msgs::MuxControl msg;
  msg.sender_id = sender_id;
  msg.switch_to_id = switch_to_id;
  pub_.publish(msg);
}


MoveBaseMuxMessage::MoveBaseMuxMessage(ros::NodeHandle& nh, std::string topic)
: nh_(nh) {
  pub_ = nh_.advertise<pandubot_msgs::MuxMessage>(topic, 5);
}

void MoveBaseMuxMessage::Publish(string sender_id, 
                                 move_base_msgs::MoveBaseGoal goal) {
  pandubot_msgs::MuxMessage msg;
  msg.header.stamp = ros::Time::now();
  msg.sender_id    = sender_id;
  msg.goal         = goal;
  pub_.publish(msg);
}

}  // namespace pandubot_publishers