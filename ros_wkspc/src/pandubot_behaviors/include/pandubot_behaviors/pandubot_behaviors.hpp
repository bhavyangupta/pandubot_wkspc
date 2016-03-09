// Copyright [2015]
#ifndef PANDUBOT_BEHAVIORS_HPP
#define PANDUBOT_BEHAVIORS_HPP

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
// #include <actionlib_msgs/GoalStatus.h>
#include "pandubot_subscribers.hpp"
#include "pandubot_publishers.hpp"
#include "pandubot_libraries/waypoint_manager.hpp"
#include "pandubot_libraries/navigation_action_client.hpp"

namespace pandubot_behaviors {
class HouseKeeping; // Forward declaration of class

/** Inherits from the navigation_action_client and adds the functionality of 
 *  non-blocking SendGoal calls. Specialised for HouseKeeping because it
 *  needs a reference of that object at construction.
 */
class GoToGoalHouseKeeping : public NavigationClient {
 private:
  bool execute_external_callback_;
  /** This pointer is not needed, but might be more "scalable in the future" */
  // void (HouseKeeping::*ExternalCallback)(void); // pointer to execute cb
  HouseKeeping& calling_object_;
 public:
  GoToGoalHouseKeeping(std::string action_name, bool new_thread, HouseKeeping& caller);
  /** Initiates a non-blocking call to the send goal. It executes a callback
   *  function passed as an argument when the goal is completed.
   */
  void SendSingleGoalWithCallbackNonBlocking(move_base_msgs::MoveBaseGoal goal);
  void DoneCb(const actionlib::SimpleClientGoalState &state,
              const move_base_msgs::MoveBaseResultConstPtr &result);
};

/** Use when you want to pass a non-class method as a callback */
class GoToGoalExternalCallback : public NavigationClient {
 private:
  bool execute_external_callback_;
  void (*external_callback_)(const actionlib::SimpleClientGoalState &state);
 public:
  GoToGoalExternalCallback(std::string action_name, bool new_thread);
  /** Initiates a non-blocking call to the send goal. It executes a callback
   *  function passed as an argument when the goal is completed.
   */
  void SendSingleGoalWithCallbackNonBlocking(move_base_msgs::MoveBaseGoal goal,
                                             void (*cb) (const actionlib::SimpleClientGoalState &state) );
  void DoneCb(const actionlib::SimpleClientGoalState &state,
              const move_base_msgs::MoveBaseResultConstPtr &result);
};


class WaypointNavigation {
 private:
  ros::NodeHandle& nh_;
  pandubot_publishers::MoveBaseMuxControl pub_nav_mux_control_;
  pandubot_publishers::MoveBaseMuxMessage pub_nav_mux_message_;
  std::string id_;
 protected:
  pandubot_libraries::WaypointManager<move_base_msgs::MoveBaseGoal> 
                                                              waypoint_manager_;                                                              
 public:
  WaypointNavigation(ros::NodeHandle& nh,
                     std::string id, 
                     std::string waypt_file, 
                     std::string mux_message_topic, 
                     std::string mux_control_topic);
  void SendNextWaypoint();
  void YieldMux(std::string switch_to_id);
};


class ReportMissingObject {
 private:
  ros::NodeHandle& nh_;
  pandubot_publishers::MoveBaseMuxControl pub_nav_mux_control_;
  pandubot_publishers::MoveBaseMuxMessage pub_nav_mux_message_;
  std::string id_;
  pandubot_libraries::WaypointManager<move_base_msgs::MoveBaseGoal> 
                                                              waypoint_manager_;
 protected:
  pandubot_subscribers::ObjectDetector     object_detector_;   
 public:
  ReportMissingObject(ros::NodeHandle& nh,
                      std::string id, 
                      std::string object_topic, 
                      std::string mux_message_topic, 
                      std::string mux_control_topic,
                      std::string waypoint_file);
  void ClaimMux();
  void YieldMux(std::string switch_to_id);
  bool WaitForObject();
  void GoHome();
};

/**
 * Complex Behavior combining object detection and waypoint navigation.
 */
class HouseKeeping {
 private:
  ros::NodeHandle &nh_;
  move_base_msgs::MoveBaseGoal next_waypoint_;
  bool goal_complete_;
  bool object_found_;
 protected:
  pandubot_libraries::WaypointManager<move_base_msgs::MoveBaseGoal>
                                                               waypoint_manager_;
  pandubot_subscribers::ObjectDetector object_detector_;
  GoToGoalHouseKeeping navigator_; 

 public: 
  HouseKeeping(ros::NodeHandle &nh, std::string move_base_action_name, 
               std::string waypoint_file, std::string object_topic,
               bool thread = true);
  void GoalCompleteCallback(); // Pass this as callback to be notified when the 
                               // Goal is done in a non-blocking way.
  void Inhibit();
  /** Returns true if an is object found at each waypoint. Else returns false*/
  bool SendNextWaypoint();
};

} // namespace pandubot_behaviors


#endif  // PANDUBOT_BEHAVIORS_HPP
