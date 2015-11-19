// Copyright [2015]
#ifndef FACE_ACTION_CLIENT_HPP
#define FACE_ACTION_CLIENT_HPP

#include <ros/ros.h>
#include <pandubot_face_detection/face_detectAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

#include "voice_publisher.hpp"

using std::string;
using actionlib::SimpleActionClient;
using actionlib::SimpleClientGoalState;
using pandubot_face_detection::face_detectResultConstPtr;
using pandubot_face_detection::face_detectAction;
using pandubot_face_detection::face_detectGoal;

typedef SimpleActionClient<face_detectAction> FaceActionClient;


enum kGenderSet {
  GENDER_DEFAULT = 0,
  GENDER_ANY     = 1,
  GENDER_MALE       ,  // Not used
  GENDER_FEMALE     ,  // Not used
  };

class FaceDetectionClient{
 private:
  ros::NodeHandle& nh_;
  FaceActionClient face_client_;
  string action_name_;
  SimpleClientGoalState result_;
  face_detectResultConstPtr result_state_ptr_;
  VoicePublisher face_talker_;
  bool current_goal_active_;
 public:
  FaceDetectionClient(ros::NodeHandle &nh,string action_name, bool new_thread);

  // Returns after single detection:
  bool SendSingleGoalAndWaitWithTimeout(kGenderSet gender_id, float timeout_sec);

  // Waits until goal is found or time out reached:
  bool SendAbsoluteGoalWithTimeout(kGenderSet gender_id, int timeout_sec);

  // Call only if SendSingleGoalAndWaitForResult returns true:
  int GetDetectedGender();

  // status:
  bool GoalActive();

  // actionlib callbacks
  void DoneCb(const actionlib::SimpleClientGoalState& state, 
              const pandubot_face_detection::face_detectResultConstPtr& result);
  void FeedbackCb(const pandubot_face_detection::face_detectFeedbackConstPtr& fb);
  void ActiveCb();
};
#endif  // FACE_ACTION_CLIENT_HPP
