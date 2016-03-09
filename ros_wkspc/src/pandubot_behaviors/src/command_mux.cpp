// Copyright [2015]

#include <ros/ros.h>
#include <string>
#include "command_mux.hpp"

using std::string;

namespace command_mux {
GoToGoalMoveBaseMux::GoToGoalMoveBaseMux(string action_name, bool new_thread, 
                                         MoveBaseMux& caller)
: NavigationClient(action_name,new_thread),
  calling_object_(caller) {
  ROS_INFO_STREAM("GoToGoalMoveBaseMux constructor");
}

void GoToGoalMoveBaseMux::SendSingleGoalWithCallbackNonBlocking(
                                            move_base_msgs::MoveBaseGoal goal) {
  if(!execute_external_callback_) {
    execute_external_callback_ = true;
    // ROS_INFO_STREAM("navigator called here.");
    ROS_INFO_STREAM(__func__);
    pandubot_utilities::PrintGoalMsg(goal);
    navigation_client_.sendGoal(goal,
                                boost::bind(&GoToGoalMoveBaseMux::DoneCb, this, _1, _2),
                                boost::bind(&NavigationClient::AcceptedCb,this),
                                boost::bind(&NavigationClient::FeedbackCb,this,_1)
                                );
  }
}

void GoToGoalMoveBaseMux::DoneCb(const actionlib::SimpleClientGoalState &state,
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

MoveBaseMux::MoveBaseMux(ros::NodeHandle &nh, string control_topic, 
                         string message_topic, string action_name)
: nh_(nh),
  current_state_(""),
  goal_complete_(true),
  sent_first_goal_(false),
  navigator_(action_name,true, *this),
  cancelled_(false) {
  sub_control_inputs_ = nh_.subscribe(control_topic, 10, 
                                      &MoveBaseMux::ControlSubscriberCallback,
                                      this); 
  sub_message_inputs_ = nh_.subscribe(message_topic, 1, 
                                      &MoveBaseMux::MessageSubscriberCallback,
                                      this);
}

void MoveBaseMux::ControlSubscriberCallback(const pandubot_msgs::MuxControl::
                                                  ConstPtr &msg) {
  ROS_INFO_STREAM(current_state_);
  if(msg->sender_id == "april") {
    current_state_ = msg->switch_to_id;
    // ROS_INFO_STREAM("To"<<current_state_); 
      // Cancel all goals as you are about to get a new goal
    if((navigator_.GetState() == actionlib::SimpleClientGoalState::PENDING) ||
       (navigator_.GetState() == actionlib::SimpleClientGoalState::ACTIVE) && !cancelled_) {
        navigator_.CancelAllGoals();
        cancelled_ = true;
      } 
  } else if (msg->sender_id == "object") {
      if ((current_state_== "") || (current_state_ == "object") || (current_state_ == "waypt")) {
        current_state_ = msg->switch_to_id;
        ROS_INFO_STREAM("To"<<current_state_);      // Cancel all goals as you are about to get a new goal
        if((navigator_.GetState() == actionlib::SimpleClientGoalState::PENDING) ||
           (navigator_.GetState() == actionlib::SimpleClientGoalState::ACTIVE) && !cancelled_) {
          navigator_.CancelAllGoals();
          cancelled_ = true;
        } 
      }
  } else if (msg->sender_id == "waypt") {
    if ((current_state_ == "") || (current_state_=="waypt"))  {
      current_state_ = msg->switch_to_id;
      ROS_INFO_STREAM("To"<<current_state_); 
      /****88************ *********************************************
      Not needed since you want this to be the default
      behavior- so it should not cancel any goals that are being executed for
      higher level behvaiours
      *******************************************************/
      // ROS_INFO_STREAM("switch_to_id = "<<msg->switch_to_id);
      // ROS_INFO_STREAM("current_state_ = "<<current_state_);
      // Cancel all goals as you are about to get a new goal
      // if(!(navigator_.GetState() == actionlib::SimpleClientGoalState::LOST)){
      //   navigator_.CancelAllGoals();
      // } else {ROS_INFO_STREAM("navigator lost!!");}
    }
  }
}

void MoveBaseMux::MessageSubscriberCallback(
                            const pandubot_msgs::MuxMessage::ConstPtr &msg) {
  // ROS_INFO_STREAM(current_state_ << " :::: " << msg->sender_id);
  if (msg->sender_id == current_state_) {
    next_goal_ = msg->goal;
    // ROS_INFO_STREAM("Send goal with sender id "<<msg->sender_id);
  }
}

/** This is executed after the current goal is completed by GoTo Behavior */
void MoveBaseMux::GoalCompleteCallback(void) {
  goal_complete_ = true;
}

void MoveBaseMux::SendNextGoal() {
  if(goal_complete_) {
    // ROS_INFO_STREAM(__func__);
    navigator_.SendSingleGoalWithCallbackNonBlocking(next_goal_);
    cancelled_ = false;
    goal_complete_ = false;
  }
}

}  // namespace command_mux