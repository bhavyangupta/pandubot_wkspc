// Copyright [2015]
#include "object_action_client.hpp"
#include "object_types.hpp"
#include "voice_publisher.hpp"
#include <pandubot_object_recognition/object_actionAction.h>
#include <string>

using actionlib::SimpleActionClient;
using actionlib::SimpleClientGoalState;
using pandubot_object_recognition::object_actionResultConstPtr;
using pandubot_object_recognition::object_actionFeedbackConstPtr;

ObjectDetectionClient::ObjectDetectionClient(ros::NodeHandle &nh, string action_name, bool new_thread)
: nh_(nh),
  object_talker_(nh),
  object_client_(action_name,new_thread),
  result_(actionlib::SimpleClientGoalState::LOST) {
  action_name_   = action_name;
  current_goal_active_ = false;
  object_client_.waitForServer();
}

bool ObjectDetectionClient::SendSingleGoalAndWaitWithTimeout(object_types::kObjectsSet object_id,
                                                             float timeout_sec) {
  // TODO(bhavya): check server connection
  // TODO(bhavya): publish feedback
  object_actionGoal goal;
  goal.target_class = static_cast<int>(object_id);
  object_client_.sendGoal(goal,
                          boost::bind(&ObjectDetectionClient::DoneCb,this,_1,_2),
                          boost::bind(&ObjectDetectionClient::ActiveCb,this),
                          boost::bind(&ObjectDetectionClient::FeedbackCb,this,_1));
  current_goal_active_ = true;
  object_client_.waitForResult(ros::Duration(timeout_sec));
  ROS_INFO_STREAM("Object got result");
  if(current_goal_active_) {
    ROS_INFO_STREAM("Object goal cancelled");
    object_client_.cancelGoal();
    object_client_.stopTrackingGoal();
  }
  return current_goal_active_;
}

bool ObjectDetectionClient::SendAbsoluteGoalWithTimeout(object_types::kObjectsSet object_id,
                                                        int timeout_sec) {
  // // TODO(bhavya): check server connection
  // // TODO(bhavya): publish feedback
  // object_actionGoal goal;
  // goal.target_class = static_cast<int>(object_id);
  // object_client_.sendGoalAndWait(goal, ros::Duration(timeout_sec));
  // result_ = object_client_.getState();
  // result_state_ptr_ = object_client_.getResult();
  // if (result_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
  //   return true;
  // } else {
  //   return false;
  // }
}

int  ObjectDetectionClient::GetDetectedClass() {
  return result_state_ptr_->detected_class;
}

void ObjectDetectionClient::DoneCb(const actionlib::SimpleClientGoalState& state,
                                   const object_actionResultConstPtr& result) {
  
  result_state_ptr_ = result;
  result_ = state;
  current_goal_active_ = false;
  return;
}

void ObjectDetectionClient::FeedbackCb(const object_actionFeedbackConstPtr& fb) {
  object_talker_.SpeakInstructions();
}

void ObjectDetectionClient::ActiveCb() {

}