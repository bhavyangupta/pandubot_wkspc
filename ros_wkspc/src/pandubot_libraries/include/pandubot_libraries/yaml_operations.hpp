#include <map>
#include <string>
#include "yaml-cpp/yaml.h"
/**
 * Atomic operator to save a single entry in the yaml file as a hashmap of
 * type <string, T>
 */
template<typename T>
void operator << (std::map<std::string,T>& output, YAML::Node& node) {
  YAML::const_iterator itr = node.begin();
  std::string key = itr->first.as<std::string>();
  T value = itr->second.as<T>();
  output[key] = value;
};
