#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>
#include "behavior_schema.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "testMotorSchema");
  ros::NodeHandle nh;
  std::string waypoint_file = "/home/bhavya/pandubot_wkspc/ros_wkspc/src/pandubot_libraries/test/testWaypoints.yaml";

  BehaviorSchema test;
  ROS_INFO_STREAM("Default" << test.GetCurrentState()); 
  test.Suppress();
  ROS_INFO_STREAM("After Suppress" << test.GetCurrentState()); 
  test.Release();
  ROS_INFO_STREAM("After Release" << test.GetCurrentState()); 

  pandubot_behavior_schemas::FollowWaypoint test_follow_waypt(nh,waypoint_file);  
  ros::Rate loop_rate(10);
  move_base_msgs::MoveBaseGoal suppressor_goal;
  suppressor_goal.target_pose.header.frame_id = "odom";
  suppressor_goal.target_pose.pose.position.x = 0;
  suppressor_goal.target_pose.pose.position.y = 0;
  suppressor_goal.target_pose.pose.position.z = 0;
  suppressor_goal.target_pose.pose.orientation.x = 0;
  suppressor_goal.target_pose.pose.orientation.y = 0;
  suppressor_goal.target_pose.pose.orientation.z = 0;
  suppressor_goal.target_pose.pose.orientation.w = 1;

  while(nh.ok()) {
    test_follow_waypt.Run();
    ros::Duration(1).sleep();
    test_follow_waypt.Suppress();
    ros::Duration(1).sleep();
    test_follow_waypt.Suppress(suppressor_goal);
    ros::Duration(1).sleep();
    test_follow_waypt.Release();
  }
}
