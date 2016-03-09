// Copyright[2015]
#include "navigation_action_client.hpp"
#include <string>

using std::string;

NavigationClient::NavigationClient(string action_name, bool new_thread)
: navigation_client_(action_name,new_thread),
  result_(actionlib::SimpleClientGoalState::LOST) {
  action_name_ = action_name;
  ROS_INFO_STREAM("Wait for move_base_server with name "<<action_name);
  navigation_client_.waitForServer();
  ROS_INFO_STREAM("Move base server found!");
}

bool NavigationClient::SendSingleGoalAndWaitForResult(float coords[], int dim) {
  // TODO(bhavya): check server connection
  // TODO(bhavya): publish feedback
  MoveBaseGoal goal = ArrayToROSmsg(coords, dim);
  navigation_client_.sendGoal(goal);
  navigation_client_.waitForResult();
  result_ =  navigation_client_.getState();
  result_state_ptr_ = navigation_client_.getResult();
  if (result_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    return false;
  }
}

bool NavigationClient::SendSingleGoalAndWaitForResult(
                                            move_base_msgs::MoveBaseGoal goal) {
  // TODO(bhavya): check server connection
  // TODO(bhavya): publish feedback
  navigation_client_.sendGoal(goal);
  navigation_client_.waitForResult();
  result_ =  navigation_client_.getState();
  result_state_ptr_ = navigation_client_.getResult();
  if (result_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    return false;
  }
}


void NavigationClient::SendSingleGoalWithCallback(
                                            move_base_msgs::MoveBaseGoal goal) {
  navigation_client_.sendGoal(goal, 
                              boost::bind(&NavigationClient::DoneCb,this,_1,_2),
                              boost::bind(&NavigationClient::AcceptedCb,this),
                              boost::bind(&NavigationClient::FeedbackCb,this,_1));
}

void NavigationClient::CancelAllGoals() {
  navigation_client_.stopTrackingGoal();
  navigation_client_.cancelGoal();
}

bool NavigationClient::SendAbsoluteGoalWithTimeout(float coords[], int timeout_s,
                                                   int dim) {
  MoveBaseGoal goal = ArrayToROSmsg(coords, dim);
  navigation_client_.sendGoalAndWait(goal, ros::Duration(timeout_s));
  result_ = navigation_client_.getState();
  result_state_ptr_ = navigation_client_.getResult();
  if (result_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    return false;
  }
}

void NavigationClient::DoneCb(const actionlib::SimpleClientGoalState &state,
                              const move_base_msgs::MoveBaseResultConstPtr 
                              &result) {  
  ROS_INFO_STREAM("Base Class: "<<__func__);
}

void NavigationClient::FeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr
                                  &feedback) {
}

void NavigationClient::AcceptedCb() {
  ROS_INFO_STREAM("New Goal to move_base");
}

actionlib::SimpleClientGoalState NavigationClient::GetState() {
  return navigation_client_.getState();
}

MoveBaseGoal NavigationClient::ArrayToROSmsg(float coords[], int dimensions) {
  MoveBaseGoal next_goal;
  next_goal.target_pose.header.frame_id = "map";
  next_goal.target_pose.pose.position.x    = coords[0];
  next_goal.target_pose.pose.position.y    = coords[1];
  next_goal.target_pose.pose.position.z    = 0;
  next_goal.target_pose.pose.orientation.x = 0;
  next_goal.target_pose.pose.orientation.y = 0;
  next_goal.target_pose.pose.orientation.z = 0;
  next_goal.target_pose.pose.orientation.w = 1;
  return next_goal;
}
