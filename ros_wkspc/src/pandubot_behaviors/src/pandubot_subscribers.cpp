// Copyright [2015]
#include "pandubot_subscribers.hpp"
#include <string>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>

using std::string;
using std::vector;
using std_msgs::Float32MultiArray;

namespace pandubot_subscribers {
ObjectDetector::ObjectDetector(ros::NodeHandle &nh,string topic_name)
: nh_(nh),
  topic_name_(topic_name),
  callback_counter_(0) {
  
  //TODO  Convert this to yaml parsing.
  object_database_[0] = "ketchup";
  object_database_[1] = "ketchup";
  object_database_[2] = "ketchup";
}

void ObjectDetector::SubscriberCallback(const Float32MultiArray::ConstPtr &msg) {
  callback_counter_++;  
  ROS_INFO_STREAM(__func__);
  detected_objects_msg_ = *msg;
}

vector<string> ObjectDetector::GetDetectedObjects() {
  vector<string> objects_detected;
  string object_name;
  if (detected_objects_msg_.data.size()) {
    for(int i = 0; i<detected_objects_msg_.data.size(); i = i+12) {
      object_name = object_database_[detected_objects_msg_.data[i]];
      objects_detected.push_back(object_name);
    }
  }
  return objects_detected;
}

int ObjectDetector::GetDetectedObjectsCount() {
  return detected_objects_msg_.data.size();
}

void ObjectDetector::StartDetection() {
  ROS_INFO_STREAM(__func__<<": Start object detection on topic "<< topic_name_);
  sub_object_ = nh_.subscribe(topic_name_, 1, &ObjectDetector::SubscriberCallback
                              , this);
}

void ObjectDetector::StopDetection() {
  ROS_INFO_STREAM("Stop object detection on topic"<<topic_name_);
  // Erase the buffer so that  you have clean state when you call StartDetection 
 // again.
  detected_objects_msg_.data.erase(detected_objects_msg_.data.begin(), detected_objects_msg_.data.end());
  sub_object_.shutdown();
}

bool ObjectDetector::CheckForObjectsWithTimeout(int timeout_s) {
  this->StartDetection();
  ros::Rate polling_rate(50);
  ros::Time start_time = ros::Time::now();
  ros::Duration time_elapsed;
  bool object_found      = false;
  vector<string> detected_objects;
  time_elapsed = ros::Time::now() - start_time;
  while((time_elapsed.toSec() < timeout_s)) {

    if(detected_objects.size()) {
      ROS_INFO_STREAM("Object Found");
      object_found = true;
      break;
    }
    ros::spinOnce(); // Ensure that the object detector callback is called.
    polling_rate.sleep();
    time_elapsed = ros::Time::now() - start_time;
  }
  this->StopDetection();
  if(!object_found) {
    ROS_INFO_STREAM("Object not found after " << timeout_s << " s");
    return false;
  } else { return true;  }
}

AprilTagDetector::AprilTagDetector(ros::NodeHandle &nh, string topic_name)
: nh_(nh),
  topic_name_(topic_name) {
}

void AprilTagDetector::SubscriberCallback(const april_tag::AprilTagList::
                                          ConstPtr &msg) {  
  detected_tags_.push_back(msg->april_tags[0]);
}

int AprilTagDetector::GetDetectedTagId() {
  // Get the most recent tag and erase the buffer.
  if (!detected_tags_.empty()) {
    int return_id = detected_tags_.at(detected_tags_.size() - 1).id;
    detected_tags_.erase(detected_tags_.begin(), detected_tags_.end());
    return return_id;
  } else return 0;
}

void AprilTagDetector::StartDetection() {
  sub_april_tag_ = nh_.subscribe(topic_name_, 5, &AprilTagDetector::
                                 SubscriberCallback, this);
  ROS_INFO_STREAM("Start april tag detection on topic "<< topic_name_);
}

void AprilTagDetector::StopDetection() {
  ROS_INFO_STREAM("Stop april tag detection on topic "<< topic_name_);
 // Erase the buffer so that  you have clean state when you call StartDetection 
 // again.
  if (!detected_tags_.empty()) {
    detected_tags_.erase(detected_tags_.begin(),detected_tags_.end());
  }
  sub_april_tag_.shutdown();
} 

GoalResultListener::GoalResultListener(ros::NodeHandle& nh, string topic)
: nh_(nh),
  goal_reached_(false) {
  sub_move_base_goal = nh_.subscribe(topic, 5, 
                                     &GoalResultListener::SubscriberCallback,
                                     this);
}

void GoalResultListener::SubscriberCallback(
                    const move_base_msgs::MoveBaseActionResult::ConstPtr& msg) {
  if(msg->status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
    goal_reached_ = true; 
    ROS_INFO_STREAM(__func__ << "Goal Completed");
  } else { goal_reached_ = false;}
}

bool GoalResultListener::IsGoalCompleted() {
  return goal_reached_;
}

void GoalResultListener::ResetGoalFlag() {
  goal_reached_ = false;
}

}  // namespace pandubot_subscribers