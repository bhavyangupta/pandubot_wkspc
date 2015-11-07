// Copyright [2015]
#ifndef OBJECT_ACTION_CLIENT_HPP
#define OBJECT_ACTION_CLIENT_HPP

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

enum kObjectsSet {
  OBJECT_DEFAULT  = 0,
  OBJECT_ROOT_BEER,
};

class ObjectDetectionClient {
 private:
  ObjectActionClient          object_client_;
  string                      action_name_;
  SimpleClientGoalState       result_;
  object_actionResultConstPtr result_state_ptr_;
 public:
  ObjectDetectionClient(string action_name, bool new_thread);

  // Returns after single detection:
  bool SendSingleGoalAndWaitForResult(kObjectsSet object_id);

  // Waits until goal is found or time out reached:
  bool SendAbsoluteGoalWithTimeout(kObjectsSet object_id, int timeout_sec);

  // Call only if SendSingleGoalAndWaitForResult returns true:
  int  GetDetectedClass();
};

#endif  // OBJECT_ACTION_CLIENT_HPP
