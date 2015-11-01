cmake_minimum_required(VERSION 2.8.3)
project(face_recognition)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package(CATKIN_DEPENDS sensor_msgs) 
catkin_python_setup()
catkin_install_python(PROGRAMS src/face_detection_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})