#include <ros/ros.h>
#include "pandubot_behaviors.hpp"
#include <string>

using std::string;

int main(int argc, char **argv) {
  ros::init(argc, argv, "waypt_behavior");
  ros::NodeHandle nh;
  string waypoints_file = "/home/bhavya/pandubot_wkspc/ros_wkspc/src/pandubot_behaviors/test/testWaypoints.yaml";
  pandubot_behaviors::WaypointNavigation waypt_behavior(nh, "waypt", 
                                                        waypoints_file,
                                                        "mux_message",
                                                        "mux_control"
                                                        );
  ros::Rate loop_rate(10);
  while(nh.ok()) {
    waypt_behavior.SendNextWaypoint();
    loop_rate.sleep();
  }
  return 0;
}