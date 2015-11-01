#include "read_yaml.hpp"

CommandParser::CommandParser(string filename)
: filename_(filename) 
{ 
  root_ = YAML::LoadFile(filename_);
}


map<string, vector<float> > CommandParser::ConvertTaskToExecutiveGoal(pair<string,string> task) {
  string predicate = task.first;
  string subject = task.second;
  map<string, vector<float> > return_goal;
  bool predicate_found = false;
  bool subject_found = false;
  for(YAML::const_iterator root_itr = root_.begin(); root_itr!=root_.end(); root_itr++){
    string command_primitive = root_itr->first.as<string>();
    if(command_primitive==predicate){
      predicate_found = true;
      YAML::Node possible_goals = root_[command_primitive];
      for(YAML::const_iterator goal_itr=possible_goals.begin(); goal_itr!=possible_goals.end(); goal_itr++){
        YAML::Node goal = *goal_itr; // matched associative array in YAML file
        YAML::const_iterator goal_map = goal.begin(); // use this to get key for the array 
        string goal_name = goal_map->first.as<string>();
        if(goal_name==subject){
          subject_found = true;
          return_goal << goal;//overloaded operator see header file
          return return_goal;
        }
      }
    } else {continue;}
  
  }
  
}
