#include <ros/ros.h>
#include <string>
#include "pandubot_msgs/MuxControl.h"

std::string current_state = "";

// void change_current_state(std::string new_state) {
//   switch (current_state) {
//     case "april"  :
//       current_state
//       break;
//     case "object" :
//       break;
//     case "waypt"  :
//       break;
//   }
// }

void callback(const pandubot_msgs::MuxControl::ConstPtr& msg) {
  if (msg->sender_id == "april" ) {  // Change at all times
      current_state = msg->switch_to_id;
  } else if (msg->sender_id == "object") { // Change only when not in april tag state
      if (current_state == "object" || current_state == "waypt" || current_state == "") {
        current_state = msg->switch_to_id;
      }
  } else if (msg->sender_id == "waypt" || current_state == "") { // Change only when in waypt mode already.
      if (current_state == "waypt") {
        current_state = msg->switch_to_id;
      }
  }
}

int main(int argc, char **argv) {
  ros::init(argc,argv,"mux_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);
  ros::Subscriber sub = nh.subscribe("mux_input", 10, &callback);

  while(nh.ok()) {
    ROS_INFO_STREAM(current_state);
    ros::spinOnce();
    loop_rate.sleep();
  }
}