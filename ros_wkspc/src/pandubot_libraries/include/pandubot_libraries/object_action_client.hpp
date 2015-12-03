// Copyright [2015]
#ifndef OBJECT_ACTION_CLIENT_HPP
#define OBJECT_ACTION_CLIENT_HPP
#include  "voice_publisher.hpp"
#include "object_types.hpp"
#include <ros/ros.h>
#include <pandubot_object_recognition/object_actionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

using std::string;
using actionlib::SimpleActionClient;
using actionlib::SimpleClientGoalState;
using pandubot_object_recognition::object_actionResultConstPtr;
using pandubot_object_recognition::object_actionAction;
using pandubot_object_recognition::object_actionGoal;

typedef SimpleActionClient<object_actionAction> ObjectActionClient;

class ObjectDetectionClient {
 private:
  ros::NodeHandle&            nh_;
  ObjectActionClient          object_client_;
  string                      action_name_;
  SimpleClientGoalState       result_;
  object_actionResultConstPtr result_state_ptr_;
  VoicePublisher object_talker_;
  bool current_goal_active_;
 public:
  ObjectDetectionClient(ros::NodeHandle &nh, string action_name, bool new_thread);

  // Returns after single detection:
  bool SendSingleGoalAndWaitWithTimeout(object_types::kObjectsSet object_id, float timeout_sec);

  // Waits until goal is found or time out reached:
  bool SendAbsoluteGoalWithTimeout(object_types::kObjectsSet object_id, int timeout_sec);

  // Call only if SendSingleGoalAndWaitForResult returns true:
  int  GetDetectedClass();

  //actionlib callbacks
  void DoneCb(const actionlib::SimpleClientGoalState& state,
              const pandubot_object_recognition::object_actionResultConstPtr& result);
  void FeedbackCb(const pandubot_object_recognition::object_actionFeedbackConstPtr& fb);
  void ActiveCb();
};

#endif  // OBJECT_ACTION_CLIENT_HPP
