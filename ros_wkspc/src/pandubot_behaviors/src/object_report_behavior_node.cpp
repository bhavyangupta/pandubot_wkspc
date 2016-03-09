#include <ros/ros.h>
#include "pandubot_behaviors.hpp"
#include <string>

using std::string;

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_report_behavior");
  ros::NodeHandle nh;
  string waypoints_file = "/home/bhavya/pandubot_wkspc/ros_wkspc/src/pandubot_behaviors/test/testWaypoints.yaml";
  pandubot_behaviors::ReportMissingObject object_reporter(nh, "object",
                                                          "objects",
                                                          "mux_message",
                                                          "mux_control",
                                                          waypoints_file);
  ros::Rate loop_rate(20);
  pandubot_subscribers::GoalResultListener goal_result_listener(nh,
                                                                 "/move_base/result");
  while(nh.ok()) {
    if (goal_result_listener.IsGoalCompleted()) {
      object_reporter.ClaimMux();
      goal_result_listener.ResetGoalFlag();
      if(!object_reporter.WaitForObject()) {
        object_reporter.YieldMux("waypt");
      }
      // if (!object_reporter.WaitForObject()) {
        // object_reporter.GoHome();
      // }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

}