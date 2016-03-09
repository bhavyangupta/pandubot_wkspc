#include "pandubot_behaviors.hpp"
#include "pandubot_subscribers.hpp"
#include <ros/ros.h>
#include <string>

using std::string;

int main(int argc, char **argv) {
  ros::init(argc, argv, "testPandubotBehaviors");
  ros::NodeHandle nh;
  string waypoints_file = "/home/bhavya/pandubot_wkspc/ros_wkspc/src/pandubot_behaviors/test/testWaypoints.yaml";
  string detected_objects_topic = "objects";
  string april_tags_topic = "april_tags";
  string move_base_action = "move_base";
  pandubot_behaviors::HouseKeeping housekeeper(nh,
                                               move_base_action, 
                                               waypoints_file, 
                                               detected_objects_topic);
  pandubot_subscribers::AprilTagDetector tag_detector(nh, april_tags_topic);
  tag_detector.StartDetection();
  ros::Rate loop_rate(10);
  while(nh.ok()) {
    // ROS_INFO_STREAM(__func__);
    if(tag_detector.GetDetectedTagId()) {
      ROS_INFO_STREAM(__func__<<" Found April Tag "<<tag_detector.GetDetectedTagId() );
      housekeeper.Inhibit();
      // CALL HERE
      tag_detector.StopDetection();
    } 
    
    if (housekeeper.SendNextWaypoint()) {
    } else {
      ROS_INFO_STREAM(__func__<< " SendNextWaypoint returned false. Go Home." );
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}