// Copyright [2015]
#include "speech_synthesiser.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

using std::string;
using std_msgs::String;

Synthesiser::Synthesiser(ros::NodeHandle& nh, string topic)
:nh_(nh),
synthesiser_input_topic_(topic) {
  speech_pub_ = nh_.advertise<String>(synthesiser_input_topic_, 5);
  ROS_INFO_STREAM("synth const");
}

void Synthesiser::publish_string(string speak_text) {
  String rosmsg = text_to_rosmsg(speak_text);
  speech_pub_.publish(rosmsg);
  ROS_INFO_STREAM("sent " << speak_text);
  ROS_INFO_STREAM("sent " << rosmsg.data);
  ros::spinOnce();
}

string Synthesiser::rosmsg_to_text(String rosmsg) {
  return rosmsg.data;
}

String Synthesiser::text_to_rosmsg(string text) {
  String rosmsg;
  rosmsg.data = text;
  return rosmsg;
}

