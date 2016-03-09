// Copyright [2015]
#include <ros/ros.h>
#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "behavior_schema.hpp"
#include "motor_schema.hpp"
#include "perceptual_schema.hpp"
#include "utilities.hpp"
#include "object_action_client.hpp"

using move_base_msgs::MoveBaseGoal;
using actionlib::SimpleClientGoalState;

namespace pandubot_behavior_schemas {
FollowWaypoint::FollowWaypoint(ros::NodeHandle &nh, std::string filename,
                               std::string action_name)
: nh_(nh),
  waypoint_manager_(filename),
  navigator_(nh,action_name) {}

void FollowWaypoint::Suppress(MoveBaseGoal suppressor_goal) {
  // TODO: First cancel all goals here.
  ROS_WARN("Behavior Suppressed");
  current_state_   = SUPPRESSED;
  suppressor_goal_ = suppressor_goal;  
}
/**
 * Sends a new waypoint provided by the waypoint manager provided the action 
 * server is not currently in the ACTIVE state and the behavior is released.
 * If the behavior is suppressed, then the suppressor goal is sent.
 */
void FollowWaypoint::Run() {
   MoveBaseGoal current_waypt;
  if (current_state_ == RELEASED) {
    if(navigator_.GetState() != SimpleClientGoalState::ACTIVE) {
      ROS_INFO_STREAM(__func__ << "Send new Waypoint");
      current_waypt = waypoint_manager_.GetNextWaypoint();
    }
  } else if (current_state_ == SUPPRESSED) {
    ROS_INFO_STREAM(__func__ << "Send suppressor goal");
    current_waypt = suppressor_goal_;
  }
  pandubot_utilities::PrintGoalMsg(current_waypt);
  navigator_.TrackGoal(current_waypt);
}

} // pandubot_behavior_schemas
