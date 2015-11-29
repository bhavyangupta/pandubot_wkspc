// Copyright [2015]
#include <ros/ros.h>
#include "table_monitor.hpp"

int main(int argc, char** argv) {
  ros::init(argc,argv,"test_TableMonitor");
  ros::NodeHandle nh;
  TableMonitor testMonitor("/home/bhavya/pandubot_wkspc/ros_wkspc/src/pandubot_types/test/test_TableMonitor.yaml");
}
