// Copyright [2015]
#include "object_action_client.hpp"
#include <string>

ObjectDetectionClient::ObjectDetectionClient(string action_name, bool new_thread)
: object_client_(action_name,new_thread),
  result_(actionlib::SimpleClientGoalState::LOST) {
  action_name_   = action_name;
  object_client_.waitForServer();
}

bool ObjectDetectionClient::SendSingleGoalAndWaitForResult(kObjectsSet object_id) {
  // TODO(bhavya): check server connection
  // TODO(bhavya): publish feedback
  object_actionGoal goal;
  goal.target_class = static_cast<int>(object_id);
  object_client_.sendGoal(goal);
  object_client_.waitForResult();
  result_ = object_client_.getState();
  result_state_ptr_ = object_client_.getResult();
  if (result_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    return false;
  }
}

bool ObjectDetectionClient::SendAbsoluteGoalWithTimeout(kObjectsSet object_id,
                                                        int timeout_sec) {
  // TODO(bhavya): check server connection
  // TODO(bhavya): publish feedback
  object_actionGoal goal;
  goal.target_class = static_cast<int>(object_id);
  object_client_.sendGoalAndWait(goal, ros::Duration(timeout_sec));
  result_ = object_client_.getState();
  result_state_ptr_ = object_client_.getResult();
  if (result_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    return false;
  }
}

int  ObjectDetectionClient::GetDetectedClass() {
  return result_state_ptr_->detected_class;
}
