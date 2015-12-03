// Copyright [2015]
#ifndef WAYPOINT_MANAGER_HPP
#define WAYPOINT_MANAGER_HPP

#include "yaml-cpp/yaml.h"
#include "utilities.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>

/**
 * Class to handle fetching and updating waypoints. Abstracts away the file 
 * reading mess from the caller. Provides a simple interface to read waypoints
 * sequentially from the file passed in the constructor. The template parameter
 * is the type of the waypoint that the manager returns. Note that the yaml 
 * file can be same for different return types (See constructor for details).
 * It maintains two indices: the index of the current waypoint and that of the 
 * next waypoint.
 * There is waypoint magic that is not implemented right now, but can be done 
 * later (like shuffling to mimic "wandering behaior").
 * 
 */

template <typename WayptType>
class WaypointManager {
 protected:
  std::vector<WayptType> waypoints_;
  std::string frame_id_;
  int number_of_waypoints_;
  int current_waypt_idx_;
  int next_waypt_idx_;
  /** If true, the indices reset after they reach the end of waypoints. */
  bool loop_infinitely_;
  
  /**
   *  Default implementation for MoveBaseGoal return type. 
   *  Derive a subclass and override this if a different return type is needed. 
   */
  virtual void ParseYAML(std::string filename) {
    // TODO: Implement this
    YAML::Node root = YAML::LoadFile(filename);
    YAML::Node frame_node = root[0];
    frame_id_ = frame_node["frame_id"].as<std::string>();

    ROS_INFO_STREAM("Frame: " << frame_id_);
    std::vector<float> waypoint;
    YAML::Node waypt_node;
    WayptType waypt_msg;
    for (int i = 1; i < root.size(); i++) {
      waypt_node = root[i];
      waypoint = waypt_node.as<std::vector<float> >();
      waypt_msg = pandubot_utilities::ConvertVectorToGoalMsg(waypoint,frame_id_);
      waypoints_.push_back(waypt_msg);
    }
    ROS_INFO_STREAM("Number of Waypoints " << waypoints_.size());    
  } 

 public:
  WaypointManager(std::string filename, bool loop_infinitely = true)
  : loop_infinitely_(loop_infinitely),
    current_waypt_idx_(0),
    next_waypt_idx_(1) {
    //[DONE]: read yaml here.
    ParseYAML(filename);
    number_of_waypoints_ = waypoints_.size();
  }

  /** 
   * Gets the waypoint in sequence and increments the indices. Loops back if 
   *  loop_infinitely == true 
   */
  WayptType GetNextWaypoint() {
    // [DONE]: check for overflows and reset.
    current_waypt_idx_ = next_waypt_idx_;
    next_waypt_idx_++;
    if(next_waypt_idx_ > number_of_waypoints_ - 1) { // Reached end of list.
      next_waypt_idx_ = 0;                           // Reset counters to zero.
    }
    return (waypoints_.at(current_waypt_idx_));        
  }
 
  /** 
   * Provides random access to waypoint index passed. Does not change the 
   *  indices
   */
  WayptType GetWaypointNumber(int index) {
    if (index > number_of_waypoints_ - 1) {
      ROS_ERROR("Queried waypoint index out of range. Return last waypoint.");
      return (waypoints_.at(number_of_waypoints_ - 1));
    } else {
      return (waypoints_.at(index));
    } 
  }

  /** Returns the number of waypoints in the file*/
  int GetNumberOfWaypoints() {
    return number_of_waypoints_;
  }

  int GetCurrentWaypointNumber() {
    return (current_waypt_idx_ + 1);
  }

  std::string GetFrameId() {
    return frame_id_;
  }

  void ResetWaypointIndices() {
    current_waypt_idx_ = 0;
    next_waypt_idx_ = 1;
  }
};

#endif  // WAYPOINT_MANAGER_HPP 
