cmake_minimum_required(VERSION 2.8.3)
project(pandubot_face_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS rospy
                                        sensor_msgs
                                        actionlib_msgs)
add_action_files(DIRECTORY action 
                 FILES face_detect.action)
catkin_python_setup()
generate_messages(DEPENDENCIES actionlib_msgs
                               std_msgs)
catkin_package(CATKIN_DEPENDS actionlib_msgs)
install(PROGRAMS src/face_detection_node.py src/testing_client.py 
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
