// Copyright [2015]
#include "utilities.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <vector>
#include <string>

using std::string;
using std::vector;
using geometry_msgs::PoseStamped;
using move_base_msgs::MoveBaseGoal;

namespace pandubot_utilities {
/** 
 * Converts PoseStamped to 7D vector. Ignores the timestamp in the message and
 * sequentially maps the other fields of the message to the vector elements.
 * @param  pose_msg PoseStamped message
 * @return          vector<float> ordered in this way: [px,py,pz,qx,qy,qz,qw]
 */
vector<float> ConvertPoseMsgToVector(PoseStamped pose_msg) {
  vector<float> pose_vector_7d;
  pose_vector_7d[0] = pose_msg.pose.position.x;
  pose_vector_7d[1] = pose_msg.pose.position.y;
  pose_vector_7d[2] = pose_msg.pose.position.z;
  pose_vector_7d[3] = pose_msg.pose.orientation.x;
  pose_vector_7d[4] = pose_msg.pose.orientation.x;
  pose_vector_7d[5] = pose_msg.pose.orientation.y;
  pose_vector_7d[6] = pose_msg.pose.orientation.w;
  return pose_vector_7d;
}

/**
 * Sequentially maps the vector data to the pose message fields. Leaves the 
 * header unpopulated
 * @param  pose_vector_7d vector<float> ordered in this way:
 *                        [px,py,pz,qx,qy,qz,qw]
 * @return         PoseStamped message
 */
PoseStamped ConvertVectorToPoseMsg(vector<float> pose_vector_7d) {
  PoseStamped pose_msg;
  pose_msg.pose.position.x = pose_vector_7d[0];
  pose_msg.pose.position.y = pose_vector_7d[1];
  pose_msg.pose.position.z = pose_vector_7d[2];
  pose_msg.pose.orientation.x = pose_vector_7d[3];
  pose_msg.pose.orientation.y = pose_vector_7d[4];
  pose_msg.pose.orientation.z = pose_vector_7d[5];
  return pose_msg;
}

/**
 * Sequentially maps the vector data to the MoveBaseGoal fields. Header is 
 * unpopulated and its upto the callee to fill it in.
 * @param  pose_7d   vector<float> ordered in this way:
 *                        [px,py,pz,qx,qy,qz,qw]
 * @param  frame_id string type frame_id. Defaults to "map"                       
 * @return          MoveBaseAction message
 */
MoveBaseGoal ConvertVectorToGoalMsg(vector<float> pose_7d, 
                                          string frame_id) {
  MoveBaseGoal goal;
  goal.target_pose.header.frame_id = frame_id;
  goal.target_pose.pose.position.x = pose_7d[0];
  goal.target_pose.pose.position.y = pose_7d[1];
  goal.target_pose.pose.position.z = pose_7d[2];
  goal.target_pose.pose.orientation.x = pose_7d[3];
  goal.target_pose.pose.orientation.y = pose_7d[4];
  goal.target_pose.pose.orientation.z = pose_7d[5];
  goal.target_pose.pose.orientation.w = pose_7d[6];
  return goal;
}

/**
 * Prints a MoveBaseGoal type message on the terminal.
 * @param msg Message to be printed.
 */
void PrintGoalMsg(MoveBaseGoal msg) {
  ROS_INFO_STREAM("Frame: " << msg.target_pose.header.frame_id);
  ROS_INFO_STREAM("x    : " << msg.target_pose.pose.position.x);
  ROS_INFO_STREAM("y    : " << msg.target_pose.pose.position.y);
  ROS_INFO_STREAM("z    : " << msg.target_pose.pose.position.z);
  ROS_INFO_STREAM("qx   : " << msg.target_pose.pose.orientation.x);  
  ROS_INFO_STREAM("qy   : " << msg.target_pose.pose.orientation.y);  
  ROS_INFO_STREAM("qz   : " << msg.target_pose.pose.orientation.z);  
  ROS_INFO_STREAM("qw   : " << msg.target_pose.pose.orientation.w);  
}

} // pandubot_utilities