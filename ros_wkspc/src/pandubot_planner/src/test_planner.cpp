#include "sentence.hpp"
#include "read_yaml.hpp"
#include <ros/ros.h>
#include <iostream>
using std::cout;
using std::endl;
int main(int argc, char**argv) {
  ros::init(argc,argv,"pandubot_planner");
  ros::NodeHandle nh;
  
  string filename = "/home/bhavya/pandubot_wkspc/ros_wkspc/src/pandubot_planner/test.yaml";
  CommandParser speech_parser(filename) ;
  
  Sentence test1("goto dustbin");
  vector<pair<string,string> > stored;
  stored = test1.GetSubjectPredicates();
  pair<string,string> value = stored[0];
  map<string, vector<float> > coordinate_map;
  coordinate_map = speech_parser.ConvertTaskToExecutiveGoal(value);
  vector<float> coordinate  = coordinate_map[value.second];
  cout<< coordinate[0] <<" "<<coordinate[1]<<" "<<coordinate[2]<<endl;

  string one = value.first;
  string two = value.second;
  cout<<one<<" " << two<<endl;
  Sentence test2("goto x and get y");
  stored = test2.GetSubjectPredicates();
  cout<<stored[0].first<<" "<<stored[0].second<<endl;
  cout<<stored[1].first<<" "<<stored[1].second<<endl;
  return 0;
}
