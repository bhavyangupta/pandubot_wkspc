#include "command_mux.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mux_node");
  ros::NodeHandle nh;
  command_mux::MoveBaseMux mux(nh, "mux_control", "mux_message", "move_base");
  ros::Rate loop_rate(10);
  while(nh.ok()) {
    mux.SendNextGoal();
    ros::spinOnce();
    loop_rate.sleep();
  } 
}
