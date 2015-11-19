 // Copyright[2015]
#include <string>
#include "speech_synthesiser.hpp"
#include <pandubot_libraries/navigation_action_client.hpp>
#include <pandubot_libraries/face_action_client.hpp>
#include <pandubot_libraries/object_action_client.hpp>
#include <pandubot_libraries/voice_publisher.hpp>
#include <sound_play/sound_play.h>

using std::string;

int main(int argc, char**argv) {
  ros::init(argc, argv, "test_executive_node");
  ros::NodeHandle nh;
  bool new_thread = true;
  string speech_synthesiser_topic = "synth_out";
  string move_base_action = "move_base";
  string face_detect_action = "pandubot_face_detection";
  string object_detect_action = "pandubot_object_detection";

  Synthesiser           speech_synthesiser(nh, speech_synthesiser_topic);
  NavigationClient      navigation_client(move_base_action, new_thread);
  FaceDetectionClient   face_detect_client(nh,face_detect_action, new_thread);
  ObjectDetectionClient object_detect_client(nh, object_detect_action, new_thread);
  VoicePublisher        pandu_talk(nh);

  float home[]            = {0, 0, 0};
  float vending_machine[] = {22.037, -15.260, 0};
  float bump_space[]      = {-4.171, 0, 0};
  //-15.256, 11.429, 0.000), Orientation(0.000, 0.000, 0.676, 0.737
 // at 1st turn heading to bunk space(facing charity's office):Position(-15.257, 1.147, 0.000), Orientation(0.000, 0.000, 1.000, 0.020) = Angle: 3.103

  kGenderSet face_detect_target = GENDER_ANY;
  kObjectsSet object_recog_target = OBJECT_PEPSI; // int 4

  ros::Duration timeout_face_s(10);
  ros::Duration timeout_object_s(10);
  ros::Time task_start_time = ros::Time::now();
  ros::Time task_elapsed_time = ros::Time::now();
  bool timeout_reached = false;
  //face_detect_target = FaceDetectionClient::kGenderSet::ANY; 
  //object_recog_target = ObjectDetectionClient::kObjectsSet::ROOT_BEER; // int 1
  int face_timeout_s = 20;
  float object_timeout_s = 20;
  // Go to bump space
  ROS_INFO_STREAM("Going to Bump space");
  pandu_talk.SpeakString("Check statement");
  pandu_talk.SpeakHello();

  int repeat_count = 1;
  while (repeat_count < 2) {
    pandu_talk.SpeakString("I am going to bump space");
    pandu_talk.SpeakString("To get you a pepsi ");
    navigation_client.SendSingleGoalAndWaitForResult(bump_space);
    // face detect

    ROS_INFO_STREAM("Detecting face"); 
    timeout_reached = face_detect_client.SendSingleGoalAndWaitWithTimeout(face_detect_target, face_timeout_s);
    if(timeout_reached){
      pandu_talk.SpeakString("Task timed out");
      pandu_talk.SpeakString("Mission Failed");
      pandu_talk.SpeakGoHome(); 
    }
    else {
      pandu_talk.SpeakHello();
      pandu_talk.SpeakRequestObject("pepsi");
      pandu_talk.SpeakInstructions();
      ROS_INFO_STREAM("Detecting object");
      timeout_reached =  object_detect_client.SendSingleGoalAndWaitWithTimeout(object_recog_target, object_timeout_s);
      if (timeout_reached) {
        pandu_talk.SpeakString("Task timed out");
        pandu_talk.SpeakString("Mission Failed");   
        pandu_talk.SpeakGoHome(); 
      }
      else {
        ROS_INFO_STREAM("Going Home");
        pandu_talk.SpeakPlaceOnTray();
        pandu_talk.SpeakThankYou();
       // sound_play::SoundClient sound_client_;
        // sound_client_.playWave("/home/bhavya/Desktop/theme1.wav");
      }
    }
    navigation_client.SendSingleGoalAndWaitForResult(home);
    repeat_count++;
  }
  return 0;
}

