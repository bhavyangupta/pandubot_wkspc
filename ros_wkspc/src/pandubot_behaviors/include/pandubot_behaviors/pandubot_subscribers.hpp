// Copyright [2015
#ifndef PANDUBOT_SUBSCRIBERS_HPP
#define PANDUBOT_SUBSCRIBERS_HPP

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>
#include <vector>
#include "april_tag/AprilTagList.h"
#include "april_tag/AprilTag.h"

/**
 * These class wrap the message subscriber for various detectors and provide 
 * only the useful information via a method call.
 * TODO: They can be templatized in the future.
 */
namespace pandubot_subscribers {
class ObjectDetector {
 private:
  ros::NodeHandle &nh_;
  ros::Subscriber sub_object_;
 protected:
  std_msgs::Float32MultiArray detected_objects_msg_;
  int callback_counter_;
  std::map <int,std::string> object_database_;
  std::string topic_name_;
 public:
  ObjectDetector(ros::NodeHandle &nh, std::string topic_name);
  void SubscriberCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
  std::vector<std::string> GetDetectedObjects();
  int GetDetectedObjectsCount();
  /** Object detector blocks executing thread until object is found or timeout
   *  is elapsed. Returns true if object is found and false if it isn't.
   */
  bool CheckForObjectsWithTimeout(int timeout_s);
  void StartDetection();
  void StopDetection();
};

class AprilTagDetector {
 private:
  ros::NodeHandle &nh_;
  ros::Subscriber sub_april_tag_;
 protected:
  std::vector<april_tag::AprilTag> detected_tags_;
  std::string topic_name_;
 public:
  AprilTagDetector(ros::NodeHandle &nh, std::string topic_name);
  void SubscriberCallback(const april_tag::AprilTagList::ConstPtr &msg);
  /** Returns the most recently detected tag id and empties the vector
   *  Returns 0 if no tag is there.
   */
  int GetDetectedTagId();
  void StartDetection();
  void StopDetection();
};

class GoalResultListener {
 private:
  ros::NodeHandle& nh_;
  ros::Subscriber sub_move_base_goal;
  bool goal_reached_;
 public:
  GoalResultListener(ros::NodeHandle& nh, std::string topic);
  void SubscriberCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg);
  bool IsGoalCompleted();
  void ResetGoalFlag();
};

/** Use for gui button types. Use only with std_msgs types like String, Int32 etc */
template<typename TopicType>
class GuiButton {
 private:
  ros::NodeHandle& nh_;
  ros::Subscriber sub_button_;
  std::vector<TopicType> msg_buffer_;

 public:
  GuiButton(ros::NodeHandle& nh, std::string topic)
  : nh_(nh) {
    sub_button_ = nh_.subscribe(topic, 1, &GuiButton::SubscriberCallback, this);
  }

  /** 
   * Buffering of messages is needed to detect if a message 
   * has been received (as shown). A simpler way would've been to see if the 
   * class message variable is a default value. However, this is not an option
   * since the class is templated and the "default" value can be anything.
   **/
  void SubscriberCallback(const typename TopicType::ConstPtr& msg) {
    msg_buffer_.push_back(*msg);
  }

  /**
   * Returns the most recent message and clears the buffer. Returns default 
   * message if buffer is empty. Calling code should detect the default case 
   * to detect that the buffer is empty. 
   **/
  TopicType GetLastMsg() {
    TopicType return_msg;
    if (!msg_buffer_.empty()) {
      return_msg = msg_buffer_.at(msg_buffer_.size() - 1);
      msg_buffer_.erase(msg_buffer_.begin(), msg_buffer_.end());
    }
    return return_msg;
  }

  /** 
   * Clears current buffer and then waits for new msgs. Returns the most recent 
   * message.
   **/
  TopicType PollForMsg(int poll_rate_hz) {
    if (!msg_buffer_.empty()) {
      msg_buffer_.erase(msg_buffer_.begin(),msg_buffer_.end());
    }
    ros::Rate loop_rate(poll_rate_hz);
    while(nh_.ok() ) {
      if (!msg_buffer_.empty()) {
        TopicType return_msg = msg_buffer_.at(msg_buffer_.size() - 1);
        return return_msg;
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    
  }
};
}  // namespace pandubot_subscribers
#endif  // PANDUBOT_SUBSCRIBERS_HPP