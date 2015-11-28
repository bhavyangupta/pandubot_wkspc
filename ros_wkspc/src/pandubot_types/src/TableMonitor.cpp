// Copyright [2015]
#include "TableMonitor.hpp"
#include <geometry_msgs/PoseStamped.h>

TableMonitor::TableMonitor(string yaml_filename) {
  // TODO: Load table YAML here 
  SetTableState(1,TABLE_UNOCCUPIED);
  SetTableState(2,TABLE_UNOCCUPIED);
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
