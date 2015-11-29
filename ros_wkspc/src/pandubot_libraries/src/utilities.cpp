// Copyright [2015]
#include "utilities.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <vector>

using std::vector;
using geometry_msgs::PoseStamped;

/** 
 * Converts PoseStamped to 7D vector. Ignores the timestamp in the message and
 * sequentially maps the other fields of the message to the vector elements.
 * @param  pose_msg PoseStamped message
 * @return          vector<float> ordered in this way: [px,py,pz,ax,ay,az,aw]
 */
vector<float> utilities::ConvertPoseMsgToVector(PoseStamped pose_msg) {
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
 *                        [px,py,pz,ax,ay,az,aw]
 * @return         PoseStamped message
 */
PoseStamped utilities::ConvertVectorToPoseMsg(vector<float> pose_vector_7d) {
  PoseStamped pose_msg;
  pose_msg.pose.position.x = pose_vector_7d[0];
  pose_msg.pose.position.y = pose_vector_7d[1];
  pose_msg.pose.position.z = pose_vector_7d[2];
  pose_msg.pose.orientation.x = pose_vector_7d[3];
  pose_msg.pose.orientation.y = pose_vector_7d[4];
  pose_msg.pose.orientation.z = pose_vector_7d[5];
  return pose_msg;
}
