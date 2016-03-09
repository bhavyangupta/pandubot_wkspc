#include <ros/ros.h>
#include <string>
#include "pandubot_msgs/MuxControl.h"

int main(int argc, char **argv) {
  ros::init(argc,argv,"ObjectSenderNode");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<pandubot_msgs::MuxControl>("mux_input",3); 
  ros::Rate loop_rate(10);
  pandubot_msgs::MuxControl test_msg;
  while(nh.ok()) {
    test_msg.sender_id    = "object";
    test_msg.switch_to_id = "object";
    pub.publish(test_msg);
    loop_rate.sleep();
  }
}