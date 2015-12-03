//Copyright [2015]
#ifndef BEHAVIOUR_SCHEMA_HPP
#define BEHAVIOUR_SCHEMA_HPP

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>
#include <vector>
#include "motor_schema.hpp"
#include "perceptual_schema.hpp"
#include "waypoint_manager.hpp"

/**
 * Base Abstract class for all types of behaviors. It provides very basic 
 * functionality, namely - tracking and changing the state of the behavior.
 */
class BehaviorSchema {
 public:
  enum kBehaviorStates {
    INHIBITED = 0,
    SUPPRESSED,
    RELEASED,
    INVALID = -1
  };

  BehaviorSchema() {current_state_ = RELEASED;}
  /** 
  * Suppress acts as an Inhibitor if no arguments are provided. 
  * Overload this method in derived Behaviors such that the suppressing 
  * behavior passes motor commands for the suppressed behaviors as parameters
  * to this function. This will then act as a default inhibitor if no 
  * suppressing signal is passed in the call statement.
  */
  void Suppress(void) { 
    current_state_ = INHIBITED; 
    ROS_WARN("Behavior Inhibited");
  }
  /**
   * Release would need to be overriden in the derived classes 
   * to do something more sophisticated like resuming motor control.
   */
  virtual void Release(void) { current_state_ = RELEASED; }
  /**
   * This is the main coodination function that executes the behavior. This will 
   * be behavior dependent and hence is barebones here. 
   * Note: Overload this in all base classes.
   */
  virtual void Run(void) { ROS_INFO_STREAM("Base Run method"); }

  kBehaviorStates GetCurrentState() { return current_state_; }

 protected:
  kBehaviorStates current_state_;
};

namespace pandubot_behavior_schemas {

class FollowWaypoint : public BehaviorSchema {
 protected: 
  ros::NodeHandle &nh_;
  std::vector<move_base_msgs::MoveBaseGoal> waypoints_;
  WaypointManager<move_base_msgs::MoveBaseGoal> waypoint_manager_;

 public:
  FollowWaypoint(ros::NodeHandle &nh, std::string filename);
  using BehaviorSchema::Suppress; // Name Hiding in C++
  void Suppress(move_base_msgs::MoveBaseGoal suppressor_goal);
  void Release();
  /** Call this function in the main while loop. */
  void Run();
};


}  // pandubot_behavior_schemas

#endif  // BEHAVIOUR_SCHEMA_HPP
