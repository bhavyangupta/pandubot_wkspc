#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include "yaml-cpp/yaml.h"

using std::cout;
using std::endl;
using std::string;
using std::map;
using std::vector;
using std::pair;

// Atomic operator to save a single entry in the yaml file as a hashmap 
template<class T>
void operator << (map<string,T>& output, YAML::Node& node) {
  YAML::const_iterator itr = node.begin();
  string key = itr->first.as<string>();
  T value = itr->second.as<T>();
  output[key] = value;
};

class CommandParser {
  private:
    string filename_;
    YAML::Node root_;
  
  public: 
    CommandParser(string filename);
    map <string, vector<float> > ConvertTaskToExecutiveGoal(pair<string,string> task); 
};
