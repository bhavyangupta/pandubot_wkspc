// Copyright [2015]
#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <vector>

namespace pandubot_utilities {
  
  /**
   * Converts a 6 dimensional PoseStamped message to a 7D vector.
   * @param  pose_msg PoseStamped message
   * @return          vector<float> ordered in this way: [px,py,pz,qx,qy,qz,qw] 
   */
  std::vector<float> ConvertPoseMsgToVector(geometry_msgs::PoseStamped pose_msg);

  /**
   * Converts 7 dimensional vector to PoseMessage
   * @param  pose_7d vector<float> ordered in this way: [px,py,pz,qx,qy,qz,qw] 
   * @return         PoseStamped message with unpopulated header field.
   */
  geometry_msgs::PoseStamped ConvertVectorToPoseMsg(std::vector<float> pose_7d);

  /**
   * Converts 7 dimensional vector to a move_base goal. The goal message is
   * of type geometry_msgs::PoseStamped but has a different name.
   * @param  pose_7d vector<float> ordered in this way: [px,py,pz,qx,qy,qz,qw]
   * @param  frame   string frame id to be added to the goal message. Defaults 
   *                 to "map".
   * @return         MoveBaseGoal message with frame id filled in.
   */
  move_base_msgs::MoveBaseGoal ConvertVectorToGoalMsg(
                                                    std::vector<float> pose_7d,
                                                    std::string frame = "map"); 
  /**
   * Prints a MoveBaseGoal type message on the terminal.
   * @param msg Message to be printed.
   */
  void PrintGoalMsg(move_base_msgs::MoveBaseGoal msg);
}

#endif  // UTILITIES_HPP