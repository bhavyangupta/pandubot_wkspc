// Copyright[2015]
#ifndef VOICE_PUB_HPP
#define VOICE_PUB_HPP

#include <sound_play/sound_play.h>
#include <ros/ros.h>
#include <string>

using std::string;

class VoicePublisher {
 private:
  ros::NodeHandle &nh_;
  sound_play::SoundClient sound_client_;
  float sleep_default_s_;
 public:
  VoicePublisher(ros::NodeHandle &nh, string topic = "robotsound", 
                          float sleep_t = 2);
  void Speak(string phrase, float wait_time_s = 2);
  void SleepOK(float wait_time_s);
  void SpeakString(string phrase, float wait_time_s = 2);
  void SpeakHello();
  void SpeakThankYou();
  void SpeakGoHome();
  void SpeakRequestObject(string object, float wait_time_s = 4);
  void SpeakPlaceOnTray();
  void SpeakInstructions();
  void StopAll();
};


#endif // VOICE_PUB_HPP