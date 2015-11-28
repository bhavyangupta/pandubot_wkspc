// Copyright [2015]
#ifndef TABLE_MONITOR_HPP
#define TABLE_MONITOR_HPP

#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <string>

using std::string;
using std::map;
using geometry_msgs::PoseStamped;

/**
 * Class responsible for monitoring and reporting table states and sampling pose
 */
class TableMonitor {
 public:
  enum TABLE_STATE {
    TABLE_OCCUPIED = 0,
    TABLE_UNOCCUPIED,
    TABLE_INVALID = -1
  };

  TableMonitor(string yaml_filename);
  static void SetTableState(int id, TABLE_STATE new_state);
  static void SetTableCoordinates(int id, PoseStamped pose);
  static PoseStamped GetTableSamplingPose(int id);
  static TABLE_STATE GetTableState(int id);

 private:
  static map<int, TABLE_STATE> table_state_;
  static map<int, PoseStamped> table_sampling_pose_;
};

#endif  //  TABLE_MONITOR_HPP
