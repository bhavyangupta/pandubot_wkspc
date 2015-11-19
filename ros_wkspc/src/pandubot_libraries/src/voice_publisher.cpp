// Copyright [2015]
#include "voice_publisher.hpp"
#include <string>

VoicePublisher::VoicePublisher(ros::NodeHandle &nh, string topic, float sleep_t)
: nh_(nh),
  sound_client_(nh, topic),
  sleep_default_s_(sleep_t) {
  // sound_client_ = new sound_play::SoundClient();
}

void VoicePublisher::SpeakString(string phrase, float wait_time_s) {
  sound_client_.say(phrase);
  this->SleepOK(wait_time_s);
}

void VoicePublisher::SpeakHello() {
  this -> SpeakString("Hello Human", sleep_default_s_);
  this -> SpeakString("My name is Pandu", sleep_default_s_);
}

void VoicePublisher::SpeakThankYou() {
  this -> SpeakString("Mission Accomplished", sleep_default_s_);
}

void VoicePublisher::SpeakGoHome() {
  this -> SpeakString("I am going home", sleep_default_s_);  
}

void VoicePublisher::SpeakRequestObject(string object, float wait_time_s) {
  string prefix = "Please give me a ";
  this -> SpeakString(prefix + object, wait_time_s);
}

void VoicePublisher::SpeakPlaceOnTray() {
  this -> SpeakString("Please place it on the tray"); 
}

void VoicePublisher::SpeakInstructions() {
  this -> SpeakString("Show it in front of the camera", sleep_default_s_);
  this -> SpeakString("So that I can verify", sleep_default_s_);
}

void VoicePublisher::SleepOK(float wait_time_s) {
  if (nh_.ok()) {
    ros::Duration(wait_time_s).sleep();
  }
}

void VoicePublisher::StopAll() {
  sound_client_.stopAll();
}

