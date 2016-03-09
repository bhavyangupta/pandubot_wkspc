#include <ros/ros.h>
#include "command_mux.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv,"testMoveBaseMultiplexing");
  ros::NodeHandle nh;
  command_mux::MoveBaseMux test_mux(nh, "mux_input", "mux_output");
  test_mux.InitBehavior_1("b1",0.1, 0);
  test_mux.InitBehavior_2("b2",0.1, 1); // Higher priority. 
}