// Copyright [2015]
#include "pandubot_behaviors.hpp"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <string>
#include <vector>
#include "pandubot_libraries/utilities.hpp"
#include "pandubot_libraries/voice_publisher.hpp"

using std::string;
using std::vector;

namespace pandubot_behaviors {
GoToGoalHouseKeeping::GoToGoalHouseKeeping(string action_name, bool new_thread, HouseKeeping& caller)
: NavigationClient(action_name,new_thread),
  calling_object_(caller) {
  ROS_INFO_STREAM("GoToGoalHouseKeeping constructor");
}

void GoToGoalHouseKeeping::SendSingleGoalWithCallbackNonBlocking(
                                            move_base_msgs::MoveBaseGoal goal) {
  
  if(!execute_external_callback_) {
    execute_external_callback_ = true;
    // ROS_INFO_STREAM("navigator called here.");
    ROS_INFO_STREAM(__func__);
    pandubot_utilities::PrintGoalMsg(goal);
    navigation_client_.sendGoal(goal,
                                boost::bind(&GoToGoalHouseKeeping::DoneCb, this, _1, _2),
                                boost::bind(&NavigationClient::AcceptedCb,this),
                                boost::bind(&NavigationClient::FeedbackCb,this,_1)
                                );
  }
}

void GoToGoalHouseKeeping::DoneCb(const actionlib::SimpleClientGoalState &state,
                      const move_base_msgs::MoveBaseResultConstPtr &result) {
  /** Execute the callback function of the caller object to signal goal 
   *  completion.
   */
  
  if(execute_external_callback_) {
    calling_object_.GoalCompleteCallback();
    ROS_INFO_STREAM("Returned from GoalCompleteCallback");
    execute_external_callback_ = false;
  }
}
/******************************************************************************/
GoToGoalExternalCallback::GoToGoalExternalCallback(string action_name, bool new_thread)
: NavigationClient(action_name,new_thread) {
  ROS_INFO_STREAM("GoToGoalExternalCallback constructor");
}

void GoToGoalExternalCallback::SendSingleGoalWithCallbackNonBlocking(
                                            move_base_msgs::MoveBaseGoal goal,
                                            void (*cb) (const actionlib::SimpleClientGoalState&)) {
  
  // if(!execute_external_callback_) {
    // execute_external_callback_ = true;
    // ROS_INFO_STREAM("navigator called here.");
    ROS_INFO_STREAM(__func__);
    external_callback_ = cb;
    pandubot_utilities::PrintGoalMsg(goal);
    navigation_client_.sendGoal(goal,
                                boost::bind(&GoToGoalExternalCallback::DoneCb, this, _1, _2),
                                boost::bind(&NavigationClient::AcceptedCb,this),
                                boost::bind(&NavigationClient::FeedbackCb,this,_1)
                                );
  // }
}

void GoToGoalExternalCallback::DoneCb(const actionlib::SimpleClientGoalState &state,
                      const move_base_msgs::MoveBaseResultConstPtr &result) {
  /** Execute the callback function of the caller object to signal goal 
   *  completion.
   */
  
  // if(execute_external_callback_) {
    external_callback_(state);
    ROS_INFO_STREAM("Returned from GoalCompleteCallback");
    // execute_external_callback_ = false;
  // }
}

/******************************************************************************/
WaypointNavigation::WaypointNavigation(ros::NodeHandle& nh,
                                       string id,
                                       string waypt_file,
                                       string mux_message_topic, 
                                       string mux_control_topic)
: nh_(nh),
  id_(id),
  pub_nav_mux_control_(nh, mux_control_topic),
  pub_nav_mux_message_(nh, mux_message_topic),
  waypoint_manager_(waypt_file,true) {
}

void WaypointNavigation::SendNextWaypoint() {
  move_base_msgs::MoveBaseGoal goal = waypoint_manager_.GetNextWaypoint();
  pub_nav_mux_control_.Publish(id_,id_);
  pub_nav_mux_message_.Publish(id_,goal);
}

void WaypointNavigation::YieldMux(string switch_to_id) {
  pub_nav_mux_control_.Publish(id_,switch_to_id);
}
/******************************************************************************/
ReportMissingObject::ReportMissingObject(ros::NodeHandle &nh,
                                         string id,
                                         string object_topic,
                                         string mux_message_topic,
                                         string mux_control_topic,
                                         string waypoint_file)
: nh_(nh),
  id_(id),
  pub_nav_mux_control_(nh, mux_control_topic),
  pub_nav_mux_message_(nh, mux_message_topic),
  object_detector_(nh, object_topic),
  waypoint_manager_(waypoint_file,false) {
}

void ReportMissingObject::ClaimMux() {
  pub_nav_mux_control_.Publish(id_,id_);
}

bool ReportMissingObject::WaitForObject() {
  int timeout_s = 10;
  bool object_found = false;
  object_found = object_detector_.CheckForObjectsWithTimeout(timeout_s);
  return object_found;
}

void ReportMissingObject::GoHome() {
  move_base_msgs::MoveBaseGoal home = waypoint_manager_.GetWaypointNumber(0);
  pub_nav_mux_control_.Publish(id_,id_);
  pub_nav_mux_message_.Publish(id_,home);
  ROS_WARN("Going Home");
}

void ReportMissingObject::YieldMux(string switch_to_id) {
  pub_nav_mux_control_.Publish(id_,switch_to_id);
}

/******************************************************************************/
HouseKeeping::HouseKeeping(ros::NodeHandle &nh, string action_name, 
                           string waypoint_filename, string object_topic,
                           bool thread) 
: nh_(nh),
  object_detector_(nh, object_topic),
  waypoint_manager_(waypoint_filename),
  navigator_(action_name, thread, *this),
  goal_complete_(true),
  object_found_(true) { // Note the * , it is needed so that 
                                            // A reference to the object is 
                                            // passed. A pointer cannot be 
                                            // converted to a reference.
  next_waypoint_ = waypoint_manager_.GetNextWaypoint();
}


/** This callback should start object detection and return only when a decision
 *  has been made about the object detection. It should stop the detection once
 *  its job is done.
 *  IMPORTANT: This is executed in a separate thread from the main function. So
 *  monitor/change state carefully here.
 */
void HouseKeeping::GoalCompleteCallback() {
  ROS_INFO_STREAM("GoalCompleteCallback of housekeeping class");
  
  // TODO: Make this a function and consider moving it to ObjectDetector.
  object_found_  = object_detector_.CheckForObjectsWithTimeout(10);
  next_waypoint_ = waypoint_manager_.GetNextWaypoint();
  goal_complete_ = true;
}

void HouseKeeping::Inhibit() {
  navigator_.CancelAllGoals();
  waypoint_manager_.ResetWaypointIndices();
  goal_complete_ = true;
  object_found_ = true;
}

bool HouseKeeping::SendNextWaypoint() {
    // ROS_INFO_STREAM("Sending non-blocking goal");
  if(goal_complete_ && object_found_) { 
    ROS_INFO_STREAM(__func__);
    navigator_.SendSingleGoalWithCallbackNonBlocking(next_waypoint_);
    goal_complete_ = false;
    return true;
  } else if (goal_complete_ && !object_found_){
    ROS_INFO_STREAM(__func__ << "Object not found. Going home");
    return false;
  } else {
    return true;
    // ROS_INFO_STREAM(__func__<< " goal complete is false ");
  }
}

} // namespace pandubot_behaviors