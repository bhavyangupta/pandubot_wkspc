#include <ros/ros.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>
#include "pandubot_behaviors.hpp"
#include "pandubot_subscribers.hpp"
#include "pandubot_libraries/navigation_action_client.hpp"
#include "pandubot_libraries/table_monitor.hpp"
#include "pandubot_libraries/utilities.hpp"
#include "pandubot_libraries/voice_publisher.hpp"
#include <string>
#include <boost/lexical_cast.hpp>
using actionlib::SimpleClientGoalState;

actionlib::SimpleClientGoalState goal_end_state = actionlib::SimpleClientGoalState::SUCCEEDED;
bool goal_complete = true;

enum kBehaviors {
  WAYPT=0,
  OBJECT,
  TAG,
  DELIVERY,
};

void goal_complete_cb(const SimpleClientGoalState& state) {
  ROS_INFO_STREAM(__func__ << " Goal complete");
  if(state == SimpleClientGoalState::SUCCEEDED) {
    goal_complete = true;
  } 
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "mainNode");
  ros::NodeHandle nh;
  std::string tag_topic    = "april_tags";
  std::string object_topic = "objects";
  std::string done_btn_topic = "PANDU_to_operator";
  std::string deliver_btn_topic = "operator_to_bhavya";
  std::string waypoint_filename =  "/home/bhavya/pandubot_wkspc/ros_wkspc/src/pandubot_behaviors/param/nav_waypoints.yaml";
  std::string action_name = "move_base";
  std::string table_filename = "/home/bhavya/pandubot_wkspc/ros_wkspc/src/pandubot_behaviors/param/table_sampling_poses.yaml";


  kBehaviors current_behavior = WAYPT;
  TableMonitor table_monitor(table_filename);
  
  pandubot_subscribers::AprilTagDetector tag_releaser(nh, tag_topic);
  pandubot_subscribers::ObjectDetector   object_releaser(nh, object_topic);
  
  pandubot_libraries::WaypointManager<move_base_msgs::MoveBaseGoal> 
                                       waypoint_manager(waypoint_filename,true);
  VoicePublisher talker(nh);

  pandubot_behaviors::GoToGoalExternalCallback navigator(action_name, true);

  pandubot_subscribers::GuiButton<std_msgs::Int32> done_btn(nh, done_btn_topic);
  pandubot_subscribers::GuiButton<std_msgs::Int32> deliver_btn(nh, 
                                                              deliver_btn_topic);
  move_base_msgs::MoveBaseGoal next_goal;
  move_base_msgs::MoveBaseGoal home_goal = waypoint_manager.GetWaypointNumber(0);
  move_base_msgs::MoveBaseGoal tag_goal;
  
  ros::Rate loop_rate(30);
  // object_releaser.StartDetection();
  tag_releaser.StartDetection();
  talker.SpeakHello();
  while(nh.ok()) {
    // Look for home call 
      // Wait for instructions 
        //Get instructions and fetch the id 
        //GoTo Table  
    std_msgs::Int32 deliver_btn_msg = deliver_btn.GetLastMsg();
    if (deliver_btn_msg.data == 999) { // Signal come to home for delivery.
      current_behavior = DELIVERY;
    }

    // Look for tags 
    int id = tag_releaser.GetDetectedTagId();
    // If valid id is found - start tag follow behavior and don't return until
    // you're done.
    if (id) {
      tag_releaser.StopDetection();
      ROS_INFO_STREAM("Tag Detected "<< id);
      // Set the parameter on the server to signal the gui for taking the 
      // correct order.
      nh.setParam("/TableNum",id);
      tag_goal = table_monitor.GetTableSamplingPose(id);
      pandubot_utilities::PrintGoalMsg(tag_goal);
      current_behavior = TAG;
      ROS_INFO_STREAM("[BEHAVIOR SWITCH] tag");
    }
    switch(current_behavior) {
      case WAYPT :
        // if(goal_complete) {
          // ROS_INFO_STREAM("w1");
          next_goal = waypoint_manager.GetNextWaypoint();   
          // ROS_INFO_STREAM("w2");
          // DOes this spawn a new thread???? Probably since the callbacks are 
          // processed in a new thread, parallely
          navigator.SendSingleGoalWithCallbackNonBlocking(next_goal,
                                                          goal_complete_cb);
          // ROS_INFO_STREAM("w3");
          goal_complete = false;
          // ROS_INFO_STREAM("w4");
          current_behavior = OBJECT;
          // ROS_INFO_STREAM("w5");
          ROS_INFO_STREAM("[BEHAVIOR SWITCH] object");
        // }
        break;
      
      case OBJECT :
        if(goal_complete) {
          // Detect objects here
          if (object_releaser.CheckForObjectsWithTimeout(7)) {
            ROS_INFO_STREAM("Object found");
          } else { 
            ROS_INFO_STREAM("Object not found. Going home for report.");
            navigator.SendSingleGoalAndWaitForResult(home_goal);
            int id = table_monitor.GetIdFromSamplingPose(next_goal);
            if (id > 0) {
              string report_msg = "There is no ketchup on table " + boost::lexical_cast<string>(id);
              ROS_INFO_STREAM(report_msg);
              talker.SpeakString(report_msg,4); 
            } else {ROS_ERROR("Goal queried is not in database. Ignoring");}
          }
          goal_complete = false;
          current_behavior = WAYPT;
          ROS_INFO_STREAM("[BEHAVIOR SWITCH] waypt");
        }
        break;
      
      case TAG :
        ROS_WARN("Going to Table");
        pandubot_utilities::PrintGoalMsg(tag_goal);
        // next_goal = waypoint_manager.GetWaypointNumber(0);
        // navigator.SendSingleGoalAndWaitForResult(next_goal);
        navigator.SendSingleGoalAndWaitForResult(tag_goal);
        // navigator.SendSingleGoalWithCallbackNonBlocking(next_goal,
        //                                                 goal_complete_cb);
        ROS_WARN("At Table");
        talker.SpeakHello();
        talker.SpeakString("I am your server for the day", 4);
        talker.SpeakString("Please use the display to order", 4);
        done_btn.PollForMsg(30);
        talker.SpeakString("Thank you for your order", 4);
        current_behavior = WAYPT;
        ROS_INFO_STREAM("[BEHAVIOR SWITCH] waypt");
        goal_complete = false;
        tag_releaser.StartDetection();
        break;

      case DELIVERY :
        ROS_INFO_STREAM("Going home for delivery");
        navigator.SendSingleGoalAndWaitForResult(home_goal);
        // Wait for message on the delivery topic
        std_msgs::Int32 deliver_btn_msg = deliver_btn.PollForMsg(30);
        next_goal = table_monitor.GetTableSamplingPose(deliver_btn_msg.data);
        navigator.SendSingleGoalAndWaitForResult(next_goal);
        ROS_INFO_STREAM("Collect your order");
        talker.SpeakString("Please collect your order",4);
        talker.SpeakString("Press done",3);
        talker.SpeakString("to send me home", 4);
        done_btn.PollForMsg(30);
        current_behavior = WAYPT;
        goal_complete = false;
        break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  tag_releaser.StopDetection();
  // object_releaser.StopDetection();
  return 0;
}


/*
 // Look for tags 
    int id = tag_releaser.GetDetectedTagId();
    // If valid id is found - start tag follow behavior and don't return until
    // you're done.
     if (id) {
      // tag_releaser.StopDetection();
      ROS_INFO_STREAM("Tag Detected "<< id);
      if(!goal_complete)  {
        // navigator.CancelAllGoals();
        ROS_WARN("Going Home");
        next_goal = waypoint_manager.GetWaypointNumber(0);
        // navigator.SendSingleGoalAndWaitForResult(next_goal);
        navigator.SendSingleGoalWithCallbackNonBlocking(next_goal,
                                                        goal_complete_cb);
        ROS_WARN("Sent Home");
        // goal_complete = true;
      }
      // tag_releaser.StartDetection();
      // Get the sampling pose from the Table Monitor here.
     } else if (goal_complete) { // Default behavior
        // Detect objects here
        // if (object_releaser.CheckForObjectsWithTimeout(10)) {
        //   ROS_INFO_STREAM("Object found");
        // } else { ROS_INFO_STREAM("Object not found");}
        next_goal = waypoint_manager.GetNextWaypoint();   
        navigator.SendSingleGoalWithCallbackNonBlocking(next_goal,
                                                        goal_complete_cb);
        goal_complete = false;
     }

    ros::spinOnce();
    loop_rate.sleep();

 */