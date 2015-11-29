// Copyright [2015]
#ifndef TABLE_MONITOR_HPP
#define TABLE_MONITOR_HPP

#include "yaml-cpp/yaml.h"
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <string>


/**
 * Class responsible for monitoring and reporting table states and sampling pose
 * Example:
 *
 */
class TableMonitor {
 public:
  enum TABLE_STATE {
    TABLE_OCCUPIED = 0,
    TABLE_UNOCCUPIED,
    TABLE_INVALID = -1
  };

  TableMonitor(std::string yaml_filename);
  static void SetTableState(int id, TABLE_STATE new_state);
  static void SetTableCoordinates(int id, geometry_msgs::PoseStamped pose);
  static geometry_msgs::PoseStamped GetTableSamplingPose(int id);
  static TABLE_STATE GetTableState(int id);

 private:
  static std::map<int, TABLE_STATE> table_state_;
  static std::map<int, geometry_msgs::PoseStamped> table_sampling_pose_;
};

#endif  //  TABLE_MONITOR_HPP