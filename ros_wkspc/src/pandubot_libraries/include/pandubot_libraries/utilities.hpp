// Copyright [2015]
#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <geometry_msgs/PoseStamped.h>
#include <vector>

namespace utilities {
  
  /**
   * Converts a 6 dimensional PoseStamped message to a 7D vector.
   * @param  pose_msg PoseStamped message
   * @return          vector<float> ordered in this way: [px,py,pz,ax,ay,az,aw] 
   */
  std::vector<float> ConvertPoseMsgToVector(geometry_msgs::PoseStamped pose_msg);

  /**
   * Converts 7 dimensional vector to PoseMessage
   * @param  pose_7d vector<float> ordered in this way: [px,py,pz,ax,ay,az,aw] 
   * @return         PoseStamped message with unpopulated header field.
   */
  geometry_msgs::PoseStamped ConvertVectorToPoseMsg(std::vector<float> pose_7d);

}

#endif  // UTILITIES_HPP