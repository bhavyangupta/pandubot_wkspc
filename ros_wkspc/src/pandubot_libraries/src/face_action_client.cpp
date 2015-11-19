// Copyright [2015]
#include "face_action_client.hpp"
#include  "voice_publisher.hpp"
#include <string>
#include <pandubot_face_detection/face_detectAction.h>
#include <actionlib/client/simple_action_client.h>

using pandubot_face_detection::face_detectResultConstPtr;
using pandubot_face_detection::face_detectFeedbackConstPtr;

using actionlib::SimpleClientGoalState;


FaceDetectionClient::FaceDetectionClient(ros::NodeHandle& nh,
                                         string action_name,
                                         bool new_thread)
: nh_(nh),
  face_talker_(nh),
  face_client_(action_name,new_thread),
  result_(actionlib::SimpleClientGoalState::LOST) {
  action_name_ = action_name;
  current_goal_active_ = false;
  face_client_.waitForServer();
}

bool FaceDetectionClient::SendSingleGoalAndWaitWithTimeout(kGenderSet gender_id,
                                                           float timeout_sec) {
  // TODO(bhavya): check server connection
  // TODO(bhavya): publish feedback
  face_detectGoal goal;
  goal.target_gender = static_cast<int>(gender_id);
  face_client_.sendGoal(goal, 
                        boost::bind(&FaceDetectionClient::DoneCb,this,_1,_2),
                        boost::bind(&FaceDetectionClient::ActiveCb,this),
                        boost::bind(&FaceDetectionClient::FeedbackCb,this,_1));
  current_goal_active_ = true;
  ROS_INFO_STREAM("waiting for result");
  face_client_.waitForResult(ros::Duration(timeout_sec));
  ROS_INFO_STREAM("got result");
  if (current_goal_active_) {
    face_client_.cancelGoal();
    face_client_.stopTrackingGoal();
  }
  return current_goal_active_;
}

/* Returns true until the goal is succeeded/aborted. Then returns false */ 
bool FaceDetectionClient::GoalActive(){
  return current_goal_active_;
}

bool FaceDetectionClient::SendAbsoluteGoalWithTimeout(kGenderSet gender_id,
                                                      int timeout_sec ) {
  // TODO(bhavya): check server connection
  // TODO(bhavya): publish feedback
  // face_detectGoal goal;
  // goal.target_gender = static_cast<int>(gender_id);
  // face_client_.sendGoalAndWait(goal, ros::Duration(timeout_sec));
  // // face_client_.waitForResult();
  // result_ = face_client_.getState();
  // result_state_ptr_ = face_client_.getResult();
  // if (result_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
  //   return true;
  // } else {
  //   return false;
  // }
  return false;
}

int FaceDetectionClient::GetDetectedGender() {
  return result_state_ptr_->detected_gender;
}

void FaceDetectionClient::DoneCb(const actionlib::SimpleClientGoalState& state,
                                 const face_detectResultConstPtr& result) {
  result_state_ptr_ = result;
  result_ = state;
  current_goal_active_ = false;
  return;
}

void FaceDetectionClient::FeedbackCb(const face_detectFeedbackConstPtr& fb){
  face_talker_.SpeakString("Can anybody help me");
}

void FaceDetectionClient::ActiveCb(){

}
