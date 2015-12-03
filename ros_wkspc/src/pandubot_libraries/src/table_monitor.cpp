// Copyright [2015]
#include "table_monitor.hpp"
#include "yaml_operations.hpp"
#include "yaml-cpp/yaml.h"
#include "utilities.hpp"
#include <move_base_msgs/MoveBaseAction.h>
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

using move_base_msgs::MoveBaseGoal;

/**
 * These variables are static and hence need to be declared here so that they
 * are initialised only once for the program. The default constructor is invoked
 * on these objects.
 */
map<int, TableMonitor::kTABLE_STATE>TableMonitor::table_state_;
map<int, MoveBaseGoal> TableMonitor::table_sampling_pose_;
string TableMonitor::frame_id_;

TableMonitor::TableMonitor(string yaml_filename) {
    YAML::Node root = YAML::LoadFile(yaml_filename);
    YAML::Node frame_node = root[0];
    int id;
    MoveBaseGoal sampling_pose_msg;
    vector<float> sampling_pose_vector;
    frame_id_ = frame_node["frame_id"].as<string>();

    ROS_DEBUG_STREAM("File Size: "<<root.size()); 
    ROS_DEBUG_STREAM("Frame: " << frame_id_);
    
    for (int i = 1 ; i<root.size(); i++) {
      YAML::Node entry = root[i];
      id = entry["id"].as<int>();
      sampling_pose_vector = entry["pose"].as<vector<float> >();
      ROS_DEBUG_STREAM(entry);
      ROS_DEBUG_STREAM(id);
      sampling_pose_msg = pandubot_utilities::
                    ConvertVectorToGoalMsg(sampling_pose_vector,frame_id_);
      table_state_[id] = TABLE_UNOCCUPIED;
      table_sampling_pose_[id] = sampling_pose_msg;
    }
}

/**
 * [TableMonitor::SetTableState Sets the state of the table id passed]
 * @param id        [int: Table id]
 * @param new_state [kTABLE_STATE: State to be assigend to table]
 */
void TableMonitor::SetTableState(int id, TableMonitor::kTABLE_STATE new_state) {
  table_state_[id] = new_state;
}

/**
 * [TableMonitor::SetTableCoordinates Sets the pose at which the robot should   
 *                                    interact with the table ]
 * @param id            [int: Table id]
 * @param sampling_pose [kTABLE_STATE: State to be assigend to table]
 */
void TableMonitor::SetTableCoordinates(int id, MoveBaseGoal sampling_pose) {
  table_sampling_pose_[id] = sampling_pose;
}

/**
 * [TableMonitor::GetTableCoordinates Returns the table coordinates in map frame]
 * @param  id [int: Table id]
 * @return    [MoveBaseGoal: Time stamped sampling pose for the table. Typically 
 *                          hardcoded and stays the same once the YAML is read]
 */
MoveBaseGoal TableMonitor::GetTableSamplingPose(int id) {
  return table_sampling_pose_[id];
}

/**
 * [TableMonitor::GetTableState Returns the table state ]
 * @param  id [int: Table]
 * @return    [kTABLE_STATE: Current State of the table]
 */
TableMonitor::kTABLE_STATE TableMonitor::GetTableState(int id) {
  return table_state_[id];
}
