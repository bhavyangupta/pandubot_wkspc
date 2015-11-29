// Copyright [2015]
#include "table_monitor.hpp"
#include "yaml_operations.hpp"
#include "yaml-cpp/yaml.h"
#include "pandubot_libraries/utilities.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <vector>
#include <map>
#include <iostream>
#include <string>

using std::string;
using std::map;
using std::cout;
using std::endl;
using std::vector;

using geometry_msgs::PoseStamped;

/**
 * These variables are static and hence need to be declared here so that they
 * are initialised only once for the program. The default constructor is invoked
 * on these objects.
 */
map<int, TableMonitor::TABLE_STATE>TableMonitor::table_state_;
map<int, PoseStamped> TableMonitor::table_sampling_pose_;

TableMonitor::TableMonitor(string yaml_filename) {
    YAML::Node root = YAML::LoadFile(yaml_filename);
    ROS_DEBUG_STREAM("Size: "<<root.size());        
    for (int i = 0 ; i<root.size(); i++) {
      YAML::Node entry = root[i];
      int id = entry["id"].as<int>();
      vector<float> sampling_pose_vector = entry["pose"].as<vector<float> >();
      ROS_DEBUG_STREAM(entry);
      ROS_DEBUG_STREAM(id);
      PoseStamped sampling_pose_msg = utilities::ConvertVectorToPoseMsg(
                                                          sampling_pose_vector);
      table_state_[id] = TABLE_UNOCCUPIED;
      table_sampling_pose_[id] = sampling_pose_msg;
    }
}

/**
 * [TableMonitor::SetTableState Sets the state of the table id passed]
 * @param id        [int: Table id]
 * @param new_state [TABLE_STATE: State to be assigend to table]
 */
void TableMonitor::SetTableState(int id, TableMonitor::TABLE_STATE new_state) {
  table_state_[id] = new_state;
}

/**
 * [TableMonitor::SetTableCoordinates Sets the pose at which the robot should   
 *                                    interact with the table ]
 * @param id            [int: Table id]
 * @param sampling_pose [TABLE_STATE: State to be assigend to table]
 */
void TableMonitor::SetTableCoordinates(int id, PoseStamped sampling_pose) {
  table_sampling_pose_[id] = sampling_pose;
}

/**
 * [TableMonitor::GetTableCoordinates Returns the table coordinates in map frame]
 * @param  id [int: Table id]
 * @return    [PoseStamped: Time stamped sampling pose for the table. Typically 
 *                          hardcoded and stays the same once the YAML is read]
 */
PoseStamped TableMonitor::GetTableSamplingPose(int id) {
  return table_sampling_pose_[id];
}

/**
 * [TableMonitor::GetTableState Returns the table state ]
 * @param  id [int: Table]
 * @return    [TABLE_STATE: Current State of the table]
 */
TableMonitor::TABLE_STATE TableMonitor::GetTableState(int id) {
  return table_state_[id];
}
