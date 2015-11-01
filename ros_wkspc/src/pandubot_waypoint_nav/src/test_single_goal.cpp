#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>

using std::cout;
using std::endl;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;

struct goal_t {
  geometry_msgs::PoseStamped goal;
  int id;
};

void callback_goal_reached(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){
  cout<<"Goal reached."<<endl;
  
}

void callback_goal_active(){
  cout<<"Goal updated."<<endl;
}

void callback_goal_status(const move_base_msgs::MoveBaseFeedbackConstPtr& curr_position){
  cout<<"Curr position(x,y): "<<endl;
  cout<<curr_position->base_position.pose.position.x<<","<<curr_position->base_position.pose.position.y<<endl;
}



int main(int argc, char**argv){
  ros::init(argc,argv,"waypoint_manager");
  ros::NodeHandle nh("~");
  goal_t home_goal;
  goal_t target_goal;
  goal_t next_goal;  
  home_goal.goal.header.frame_id = "map";
  home_goal.goal.pose.position.x = 0;
  home_goal.goal.pose.position.y = 0;
  home_goal.goal.pose.position.z = 0;
  home_goal.goal.pose.orientation.x = 0;
  home_goal.goal.pose.orientation.y = 0;
  home_goal.goal.pose.orientation.z = 0;
  home_goal.goal.pose.orientation.w = 1;
  home_goal.id = 0;

  target_goal.goal.header.frame_id = "map";
  target_goal.goal.pose.position.x = -4;
  target_goal.goal.pose.position.y = 0;
  target_goal.goal.pose.position.z = 0;
  target_goal.goal.pose.orientation.x = 0;
  target_goal.goal.pose.orientation.y = 0; 
  target_goal.goal.pose.orientation.z = 0;
  target_goal.goal.pose.orientation.w = 1;
  target_goal.id = 1;

  move_base_client ping_pong_client("move_base",true);
  ping_pong_client.waitForServer();

  next_goal = target_goal;
  move_base_msgs::MoveBaseGoal sent_goal;
  while(nh.ok()){
    sent_goal.target_pose = next_goal.goal;
    cout<<next_goal.goal.pose.position.x<<endl;
    ping_pong_client.sendGoal(sent_goal,&callback_goal_reached,&callback_goal_active,&callback_goal_status);
    ping_pong_client.waitForResult();
    if(ping_pong_client.getState()==actionlib::SimpleClientGoalState::ABORTED){
      ROS_WARN("GOAL ABORTED");
    }
    if(next_goal.id == home_goal.id){
      next_goal = target_goal;
    }
    else if(next_goal.id == target_goal.id){
      next_goal = home_goal;
    } 

    ros::spinOnce();
  }
  return 0;
}
