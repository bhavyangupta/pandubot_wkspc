// Copyright [2015]
#include <ros/ros.h>
#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include "behavior_schema.hpp"
#include "motor_schema.hpp"
#include "perceptual_schema.hpp"
#include "utilities.hpp"

using move_base_msgs::MoveBaseGoal;

namespace pandubot_behavior_schemas {
FollowWaypoint::FollowWaypoint(ros::NodeHandle &nh, std::string filename)
: nh_(nh),
  waypoint_manager_(filename) {}

void FollowWaypoint::Suppress(MoveBaseGoal suppressor_goal) {
  ROS_WARN("Behavior Suppressed");
  current_state_ = SUPPRESSED;
}

void FollowWaypoint::Release() {
  current_state_ = RELEASED;
}

void FollowWaypoint::Run() {
  waypoint_manager_.GetNextWaypoint();
  if (current_state_ == RELEASED) {
    ROS_INFO_STREAM("waypoint");
    MoveBaseGoal current_waypt = waypoint_manager_.GetNextWaypoint();
    pandubot_utilities::PrintGoalMsg(current_waypt);
  }
}

} // pandubot_behavior_schemas
