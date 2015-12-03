#include "motor_schema.hpp"

int main(int argc, char** argv) {
  ros::init(argc,argv,"testMotorSchema");
  ros::NodeHandle nh;
  pandubot_motor_schemas::GoToPose test(nh,"testAction");
}