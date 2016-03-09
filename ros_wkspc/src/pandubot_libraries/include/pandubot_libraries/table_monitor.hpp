// Copyright [2015]
#ifndef TABLE_MONITOR_HPP
#define TABLE_MONITOR_HPP

#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <map>
#include <string>


bool operator==(move_base_msgs::MoveBaseGoal& lhs, 
                move_base_msgs::MoveBaseGoal& rhs) {
  if(!(lhs.target_pose.header.frame_id == rhs.target_pose.header.frame_id)) {
    ROS_INFO_STREAM("frames dont match");
    return false;
  } else if (abs(lhs.target_pose.pose.position.x - rhs.target_pose.pose.position.x )>=0.01) {
    ROS_INFO_STREAM("pos.x");
    return false;
  } else if (abs(lhs.target_pose.pose.position.y - rhs.target_pose.pose.position.y )>=0.01) {
    ROS_INFO_STREAM("pos.y");
    return false;
  } else if (abs(lhs.target_pose.pose.position.z - rhs.target_pose.pose.position.z )>=0.01) {
    ROS_INFO_STREAM("pos.z");
    return false;
  } else if (abs(lhs.target_pose.pose.orientation.x - rhs.target_pose.pose.orientation.x )>=0.01) {
    ROS_INFO_STREAM("or.x");
    return false;
  } else if (abs(lhs.target_pose.pose.orientation.y - rhs.target_pose.pose.orientation.y )>=0.01) {
    ROS_INFO_STREAM("or.y");
    return false;
  } else if (abs(lhs.target_pose.pose.orientation.z - rhs.target_pose.pose.orientation.z )>=0.01) {
    ROS_INFO_STREAM("or.z");
    return false;
  } else if (abs(lhs.target_pose.pose.orientation.w - rhs.target_pose.pose.orientation.w )>=0.01) {
    ROS_INFO_STREAM("or.w");
    return false;
  } 
  return true;
}

/**
 * Class responsible for monitoring and reporting table states and sampling pose
 * Example:
 *
 */
class TableMonitor {
 public:
  enum kTABLE_STATE {
    TABLE_OCCUPIED = 0,
    TABLE_UNOCCUPIED,
    TABLE_INVALID = -1
  };

  TableMonitor(std::string yaml_filename);
  static void SetTableState(int id, kTABLE_STATE new_state);
  static void UpdateTableState(int id);
  static void SetTableCoordinates(int id, move_base_msgs::MoveBaseGoal pose);
  static move_base_msgs::MoveBaseGoal GetTableSamplingPose(int id);
  static kTABLE_STATE GetTableState(int id);
  static int  GetIdFromSamplingPose(move_base_msgs::MoveBaseGoal pose);

 private:
  static std::string frame_id_;
  static std::map<int, kTABLE_STATE> table_state_;
  static std::map<int, move_base_msgs::MoveBaseGoal> table_sampling_pose_;
};

#endif  //  TABLE_MONITOR_HPP