#include "motor_schema.hpp"
#include <ros/ros.h>
#include <string>

namespace pandubot_motor_schemas {
GoToPose::GoToPose(ros::NodeHandle &nh, std::string action_name)
: nh_(nh),
  action_name_(action_name) {
  ROS_INFO_STREAM("GoToPose action name: "<<action_name_);
}

bool GoToPose::TrackGoal(move_base_msgs::MoveBaseGoal goal) {
  return false;
}

}
