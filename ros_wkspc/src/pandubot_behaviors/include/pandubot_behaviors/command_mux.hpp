// Copyright [2015]
#ifndef COMMAND_MUX_HPP
#define COMMAND_MUX_HPP

#include <ros/ros.h>
#include <string>
#include <map>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "pandubot_msgs/MotorSignalMoveBase.h"
#include "pandubot_msgs/MuxControl.h"
#include "pandubot_msgs/MuxMessage.h"
#include "pandubot_libraries/navigation_action_client.hpp"
#include "pandubot_libraries/utilities.hpp"
#include "pandubot_behaviors.hpp"

namespace command_mux {
/**
 * Template type is the type of the message that the behavior outputs
 */
template<typename MsgType>
class BehaviorTopic {
 protected:
  std::string behavior_id_;
  float       timeout_s_;
  int         priority_;
  MsgType     last_msg_;

 public:
  BehaviorTopic() 
  : behavior_id_(""),
    timeout_s_(0.0),
    priority_(0) {
  }

  BehaviorTopic(std::string id, float timeout, int priority)
  : behavior_id_(id),
    timeout_s_(timeout),
    priority_(priority) {
    }

  bool isExpired() {
    ros::Duration time_elapsed = ros::Time::now() - last_msg_.header.stamp;
    return(time_elapsed.toSec() > timeout_s_);
  }
};

/**
 * CommandMultiplexer implements the multiplexing and subsumption scheme.
 * The template parameter is type of the message being multiplexed 
 * input and output types must be the same.
 */
// template<typename MsgType>
// class CommandMultiplexer {
//  protected:
//   std::map<std::string, BehaviorTopic<MsgType> > inputs_;
//   BehaviorTopic<MsgType> last_routed_topic_;
//   std::vector<BehaviorTopic <MsgType> > FindUnexpiredTopics();
//  public:
//   CommandMultiplexer(std::string input_topic, std::string output_topic)
//   /* Can be replaced with YAML parsing */
//   void AddBehavior(std::string id, float timeout, int priority) {
//     inputs_[id] = new BehaviorTopic<MsgType>(id,timeout,priority);
//   }
//   void Callback(const MsgType::ConstPtr msg);
//   ~CommandMultiplexer() {
//   }
// };

/** Inherits from the navigation_action_client and adds the functionality of 
 *  non-blocking SendGoal calls. Specialised for Comm because it
 *  needs a reference of that object at construction.
 */
class MoveBaseMux; // Forward declaration.

class GoToGoalMoveBaseMux : public NavigationClient {
 private:
  bool execute_external_callback_;
  /** This pointer is not needed, but might be more "scalable in the future" */
  // void (MoveBaseMux::*ExternalCallback)(void); // pointer to execute cb
  MoveBaseMux& calling_object_;
 public:
  GoToGoalMoveBaseMux(std::string action_name, bool new_thread, MoveBaseMux& caller);
  /** Initiates a non-blocking call to the send goal. It executes a callback
   *  function passed as an argument when the goal is completed.
   */
  void SendSingleGoalWithCallbackNonBlocking(move_base_msgs::MoveBaseGoal goal);
  void DoneCb(const actionlib::SimpleClientGoalState &state,
              const move_base_msgs::MoveBaseResultConstPtr &result);
};

/**
 * This is prototype class for checking out the multiplexing scheme. 
 * TODO: Should be derived from the CommandMultiplexer class in the future 
 */
class MoveBaseMux {
 protected:
  ros::NodeHandle &nh_;
  ros::Subscriber sub_control_inputs_;
  ros::Subscriber sub_message_inputs_;
  std::string current_state_;
  move_base_msgs::MoveBaseGoal next_goal_;
  GoToGoalMoveBaseMux navigator_;
  bool goal_complete_;
  bool sent_first_goal_;
  bool cancelled_;
  // [DONE]: Add simple action client for move base here;

 public:
  MoveBaseMux(ros::NodeHandle &nh, std::string control_topic, 
              std::string message_topic, std::string action_name);
  void ControlSubscriberCallback(const pandubot_msgs::MuxControl::ConstPtr 
                                &msg);
  void MessageSubscriberCallback(const pandubot_msgs::MuxMessage::ConstPtr
                                 &msg);
  void GoalCompleteCallback(); // Pass this as callback to be notified when the 
                               // Goal is done in a non-blocking way.
  void SendNextGoal();
};

}  // namespace command_mux

#endif  //  COMMAND_MUX_HPP
