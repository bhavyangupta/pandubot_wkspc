#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "pandubot_subscribers.hpp"
#include <string>
#include <vector>

int main(int argc, char**argv) {
  ros::init(argc, argv, "testPandubotSubscribers");
  ros::NodeHandle nh;
  pandubot_subscribers::ObjectDetector test_object_detector(nh,"objects");
  pandubot_subscribers::AprilTagDetector test_tag_detector(nh,"april_tags");
  ros::Rate loop_rate(10);
  std::vector<std::string> object_list;
  test_object_detector.StartDetection();
  int tag_id = 0;
  while (nh.ok()) {
    object_list = test_object_detector.GetDetectedObjects();
    if (object_list.size()) {
      BOOST_FOREACH(std::string object, object_list) {
        ROS_INFO_STREAM(object);
      }
    }
    
    ROS_INFO_STREAM("----");
    
    tag_id = test_tag_detector.GetDetectedTagId();
    ROS_INFO_STREAM(tag_id);

    ROS_INFO_STREAM("----");
    ros::spinOnce();
    loop_rate.sleep();
  }
  test_object_detector.StopDetection();
}