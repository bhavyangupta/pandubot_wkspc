// Copyright [2015]
#ifndef SPEECH_SYNTH_HPP
#define SPEECH_SYNTH_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

using std::string;
using std_msgs::String;

class Synthesiser{
 private:
    ros::NodeHandle &nh_;
    ros::Publisher speech_pub_;
    string synthesiser_input_topic_;

 public:
    Synthesiser(ros::NodeHandle& nh, string topic);
    void publish_string(string speak_text);
    string rosmsg_to_text(String rosmsg);
    String text_to_rosmsg(string text);
};
#endif  // SPEECH_SYNTH_HPP



