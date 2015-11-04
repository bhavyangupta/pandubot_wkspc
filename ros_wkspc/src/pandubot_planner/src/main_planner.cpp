#include <iostream>
#include <ros/ros.h>
#include "sentence.hpp"
#include "read_yaml.hpp"
//#include "pandubot_msgs/atomic_plan.h"


void callback_speech() {
  
}
int main(int argc, char**argv){
  ros::init(argc, argv,"pandubot_planner");
  ros::NodeHandle nh;
  ros::Subscriber subscribe_speech;
  
  subscribe_speech = nh.subscribe("/pandubot/speech_sentence",10,callback_speech);
  
  while(nh.ok()) {
    ros::spinOnce();
  }

  return 0;
}
