#include "motor_schema.hpp"
#include "navigation_action_client.hpp"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>

using move_base_msgs::MoveBaseGoal;
using move_base_msgs::MoveBaseResultConstPtr;
using move_base_msgs::MoveBaseFeedbackConstPtr;
using actionlib::SimpleClientGoalState;

namespace pandubot_motor_schemas {
GoToPose::GoToPose(ros::NodeHandle &nh, std::string action_name)
: nh_(nh),
  action_name_(action_name),
  navigation_client_(action_name,false) {
  ROS_INFO_STREAM("GoToPose action name: "<<action_name_);
}

void GoToPose::TrackGoal(MoveBaseGoal goal) {
  current_goal_ = goal;
  navigation_client_.sendGoal(current_goal_);
  return;
}

void GoToPose::DoneCb(const SimpleClientGoalState &state,
                      const MoveBaseResultConstPtr &result) {
  ROS_INFO_STREAM("GoToPose goal accomplished");
  return ;
}

void GoToPose::ActiveCb() {
  ROS_INFO_STREAM("GoToPose new goal");
  return ;
}

void GoToPose::FeedbackCb(const MoveBaseFeedbackConstPtr 
                                &feedback) {
  return ;
}

SimpleClientGoalState GoToPose::GetState() {
  return(navigation_client_.getState());
}
} // pandubot_motor_schemas
