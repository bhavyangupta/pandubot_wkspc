// Copyright [2015]
#ifndef TABLE_MONITOR_HPP
#define TABLE_MONITOR_HPP

#include "yaml-cpp/yaml.h"
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <map>
#include <string>


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
  static void SetTableCoordinates(int id, move_base_msgs::MoveBaseGoal pose);
  static move_base_msgs::MoveBaseGoal GetTableSamplingPose(int id);
  static kTABLE_STATE GetTableState(int id);

 private:
  static std::string frame_id_;
  static std::map<int, kTABLE_STATE> table_state_;
  static std::map<int, move_base_msgs::MoveBaseGoal> table_sampling_pose_;
};

#endif  //  TABLE_MONITOR_HPP