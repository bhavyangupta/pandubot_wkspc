// Copyright [2015]
#include "face_action_client.hpp"
#include <string>

FaceDetectionClient::FaceDetectionClient(string action_name, bool new_thread)
: face_client_(action_name,new_thread),
  result_(actionlib::SimpleClientGoalState::LOST) {
  action_name_ = action_name;
  face_client_.waitForServer();
}

bool FaceDetectionClient::SendSingleGoalAndWaitForResult(kGenderSet gender_id) {
  // TODO(bhavya): check server connection
  // TODO(bhavya): publish feedback
  face_detectGoal goal;
  goal.target_gender = static_cast<int>(gender_id);
  face_client_.sendGoal(goal);
  face_client_.waitForResult();
  result_ = face_client_.getState();
  result_state_ptr_ = face_client_.getResult();
  if (result_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    return false;
  }
}

bool FaceDetectionClient::SendAbsoluteGoalWithTimeout(kGenderSet gender_id,
                                                      int timeout_sec ) {
  // TODO(bhavya): check server connection
  // TODO(bhavya): publish feedback
  face_detectGoal goal;
  goal.target_gender = static_cast<int>(gender_id);
  face_client_.sendGoalAndWait(goal, ros::Duration(timeout_sec));
  face_client_.waitForResult();
  result_ = face_client_.getState();
  result_state_ptr_ = face_client_.getResult();
  if (result_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    return false;
  }
}

int FaceDetectionClient::GetDetectedGender() {
  return result_state_ptr_->detected_gender;
}
