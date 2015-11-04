// Copyright [2015]
#ifndef NAVIGATION_ACTION_CLIENT_HPP
#define NAVIGATION_ACTION_CLIENT_HPP

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

using std::string;
using actionlib::SimpleActionClient;
using actionlib::SimpleClientGoalState;
using move_base_msgs::MoveBaseResultConstPtr;
using move_base_msgs::MoveBaseAction;
using move_base_msgs::MoveBaseGoal;

typedef SimpleActionClient<MoveBaseAction> NavigationActionClient;

class NavigationClient {
 private:
  NavigationActionClient navigation_client_;
  string action_name_;
  SimpleClientGoalState result_;
  MoveBaseResultConstPtr result_state_ptr_;
 public:
  NavigationClient(string action_name, bool new_thread);

  // Returns after single detection:
  bool SendSingleGoalAndWaitForResult(float coords[], int dim = 3);

  // Waits until goal is found or time out reached:
  bool SendAbsoluteGoalWithTimeout(float coords[], int timeout_s, int dim = 3);

  // Call only if SendSingleGoalAndWaitForResult returns true:
  // void GetCurrentPose(float (&coords)[3]);

  MoveBaseGoal ArrayToROSmsg(float coords[], int dimensions = 3);
};

#endif  // NAVIGATION_ACTION_CLIENT_HPP
