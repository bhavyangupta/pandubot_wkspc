#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pandubot_object_recognition/object_actionAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <iostream>


using std::cout;
using std::endl;

move_base_msgs::MoveBaseGoal array_to_rosmsg(float coords[], int dimensions = 3);

int main(int argc, char ** argv) {

  ros::init(argc,argv,"test_object_waypt_node");
  bool new_thread = true;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> waypt_client
                                                      ("move_base", new_thread);
  actionlib::SimpleActionClient<pandubot_object_recognition::object_actionAction> 
                         object_client("pandubot_object_detection",new_thread);

  float home_coords[]             = {0,0,0}; 
  float vending_machine_coords[]  = {22.037,-15.260,0}; // 22.037, -15.260, 0.000
  float bump_space_coords[]       = {-5.257,0.147,0}; //-15.257, 1.147, 0.000

  move_base_msgs::MoveBaseGoal home = array_to_rosmsg(home_coords); // home
  move_base_msgs::MoveBaseGoal vending_machine = array_to_rosmsg(vending_machine_coords);// vending machine
  move_base_msgs::MoveBaseGoal bump_space = array_to_rosmsg(bump_space_coords); // some other pt

  pandubot_object_recognition::object_actionGoal root_beer;
  root_beer.target_class =  1; // right now 1 is when the picture is a root beer

  ROS_INFO_STREAM("Waiting for server");
  waypt_client.waitForServer();
  ROS_INFO_STREAM("move base server found");
  object_client.waitForServer();
  ROS_INFO_STREAM("Object server found");

  waypt_client.sendGoal(bump_space);
  ROS_INFO_STREAM("goal sent");
  waypt_client.waitForResult();
  if(waypt_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO_STREAM("REACHED GOAL"); 
    // start object action
      pandubot_object_recognition::object_actionResultConstPtr result;
      object_client.sendGoal(root_beer);
      ROS_INFO_STREAM("Looking for root beer");
      object_client.waitForResult();
    
      if(object_client.getState()==actionlib::SimpleClientGoalState::SUCCEEDED){
        result = object_client.getResult();
        if(result->detected_class == 1) {
          ROS_INFO_STREAM("FOund Root beer");
          waypt_client.sendGoal(home);
          ROS_INFO_STREAM("goal sent");
          waypt_client.waitForResult();
        }
      
      else {
        ROS_INFO_STREAM("Root beer not found. Going to vending machine");
        waypt_client.sendGoal(vending_machine);
        ROS_INFO_STREAM("goal sent");
        waypt_client.waitForResult();
        ROS_WARN("Not root beer");
      }
    }
  }

  else  {
    ROS_ERROR("WAYPOINT GOAL FAILED");
  }
  waypt_client.sendGoal(home);
  ROS_INFO_STREAM("goal sent");
  waypt_client.waitForResult();

}

move_base_msgs::MoveBaseGoal array_to_rosmsg(float coords[], int dimensions){
    //ROS_INFO_STREAM("in gen next goal"<< coords[0] << " " <<coords[1]);
    move_base_msgs::MoveBaseGoal next_goal;
    next_goal.target_pose.header.frame_id="map";
    next_goal.target_pose.pose.position.x    = coords[0] ;
    next_goal.target_pose.pose.position.y    = coords[1] ; 
    next_goal.target_pose.pose.position.z    = 0 ;
    next_goal.target_pose.pose.orientation.x = 0 ;
    next_goal.target_pose.pose.orientation.y = 0 ;
    next_goal.target_pose.pose.orientation.z = 0 ;
    next_goal.target_pose.pose.orientation.w = 1 ;
    return next_goal;
}