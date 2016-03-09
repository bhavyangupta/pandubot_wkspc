//Copyright [2015]
#ifndef MOTOR_SCHEMA_HPP
#define MOTOR_SCHEMA_HPP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>

/**
 * Template Abstract Base class for all types of motor schema. TrackGoal method 
 * must be implemented by all kinds of motor schema and is responsible for 
 * following the commanded target. The template parameter is the type of the 
 * target that a given motor schema will follow.
 */
template<typename TARGET_T>
class MotorSchema {
 protected:
  TARGET_T current_goal_;
 public:
  MotorSchema() {};
  virtual void TrackGoal(TARGET_T target) = 0;
  virtual TARGET_T GetCurrentGoal() { return current_goal_; }
};

namespace pandubot_motor_schemas {

/**
 * Motor schema specialisation for going to a particular pose using move_base
 * actionlib wrapper.
 */
class GoToPose : public MotorSchema<move_base_msgs::MoveBaseGoal> {
 protected:
  ros::NodeHandle  &nh_;
  std::string      action_name_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> navigation_client_;
  void DoneCb(const actionlib::SimpleClientGoalState &state,
              const move_base_msgs::MoveBaseResultConstPtr &result);
  void ActiveCb();
  void FeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

 public: 
  GoToPose(ros::NodeHandle &nh, std::string action_name);
  void TrackGoal(move_base_msgs::MoveBaseGoal goal);
  actionlib::SimpleClientGoalState GetState(void);
};

} // pandubot_motor_schemas
#endif  // MOTOR_SCHEMA_HPP
