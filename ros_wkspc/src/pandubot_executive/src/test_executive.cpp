// Copyright[2015]
#include <string>
#include "speech_synthesiser.hpp"
#include "navigation_action_client.hpp"
#include "face_action_client.hpp"
#include "object_action_client.hpp"

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
  FaceDetectionClient   face_detect_client(face_detect_action, new_thread);
  ObjectDetectionClient object_detect_client(object_detect_action, new_thread);

  float home[]            = {0, 0, 0};
  float vending_machine[] = {22.037, -15.260, 0};
  float bump_space[]      = {-5.257, 0.147, 0};

  int timeout_s = 15;

  kGenderSet face_detect_target = GENDER_ANY;
  kObjectsSet object_recog_target = OBJECT_ROOT_BEER;

  //face_detect_target = FaceDetectionClient::kGenderSet::ANY;
  //object_recog_target = ObjectDetectionClient::kObjectsSet::ROOT_BEER;

  // Go to charity's place
  ROS_INFO_STREAM("Going to Bump space");
  navigation_client.SendSingleGoalAndWaitForResult(bump_space);
  // face detect
  ROS_INFO_STREAM("Detecting face");
  face_detect_client.SendAbsoluteGoalWithTimeout(face_detect_target, timeout_s);
  ROS_INFO_STREAM("Publishing string");
  // speak
  speech_synthesiser.publish_string("Can you please give me a root beer?");
  ROS_INFO_STREAM("Detecting object");
  // object detect
  object_detect_client.SendAbsoluteGoalWithTimeout(object_recog_target, timeout_s);
  ROS_INFO_STREAM("Publishing string");
  // speak
  speech_synthesiser.publish_string("Thank you. I am going home");
  ROS_INFO_STREAM("Going Home");
  // go home
  navigation_client.SendSingleGoalAndWaitForResult(home);

  return 0;
}
