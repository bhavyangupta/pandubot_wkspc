//Copyright [2015]
#ifndef BEHAVIOUR_SCHEMA_HPP
#define BEHAVIOUR_SCHEMA_HPP

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>
#include <vector>
#include "table_monitor.hpp"
#include "motor_schema.hpp"
#include "perceptual_schema.hpp"
#include "waypoint_manager.hpp"
#include "object_action_client.hpp"

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

  bool isReleased() {
    return (current_state_ == RELEASED);
  }

  /**
   * Release might need to be overriden in the derived classes 
   * to do something more sophisticated like resuming motor control.
   * Call this function when you are checking for releaser outside the behavior
   * and you need to explicitly trigger a behavior.
   */
  virtual void Release(void) { current_state_ = RELEASED; }

  /**
   * This is the main coordination function that executes the behavior. This will 
   * be behavior dependent and hence is barebones here. This function does
   * nothing until the behavior is released.
   * Note: Overload this in all base classes. 
   */
  virtual void Run(void) { ROS_INFO_STREAM("Base Run method"); }

  kBehaviorStates GetCurrentState() { return current_state_; }

 protected:
  kBehaviorStates current_state_;
};

namespace pandubot_behavior_schemas {

/**
 * This is the default behavior so it does not need any releaser. 
 */
class FollowWaypoint : public BehaviorSchema {
 protected: 
  ros::NodeHandle                               &nh_;
  pandubot_libraries::WaypointManager<move_base_msgs::MoveBaseGoal> waypoint_manager_;
  pandubot_motor_schemas::GoToPose              navigator_;
  move_base_msgs::MoveBaseGoal                  suppressor_goal_;
 public:
  using BehaviorSchema::Suppress; // Name Hiding in C++
  using BehaviorSchema::Release;  
  using BehaviorSchema::isReleased;
  FollowWaypoint(ros::NodeHandle &nh, std::string filename, std::string 
                 action_name = "move_base");

  void Suppress(move_base_msgs::MoveBaseGoal suppressor_goal);
  /** Call this function in the main while loop. */
  void Run();
};

/**
 * This is a Fixed Action Pattern type behavior that takes the robot to the 
 * detected AprilTag. The behavior should only be triggered when a tag is detected.
 * 
 * Releaser:                AprilTag Detection
 * Local Perceptual Schema: None
 * Percept:                 Table Coordinates
 * Local Motor Schema:      None
 * Suppresses:              FollowWaypoint
 * Suppressed by:           None
 */
class GoToTable : public BehaviorSchema {
 protected:


 public:
  using BehaviorSchema::isReleased;
  using BehaviorSchema::Suppress;
  using BehaviorSchema::Release;
  GoToTable();
  void Run();
};
}  // pandubot_behavior_schemas

#endif  // BEHAVIOUR_SCHEMA_HPP
