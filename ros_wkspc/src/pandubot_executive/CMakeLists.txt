cmake_minimum_required(VERSION 2.8.3)
project(pandubot_executive)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        actionlib
                                        move_base_msgs
                                        pandubot_object_recognition
                                        pandubot_face_detection)
include_directories(${catkin_INCLUDE_DIRS}  ${pandubot_object_recognition_INCLUDE_DIRS} # Not really needed since
${pandubot_face_detection_INCLUDE_DIRS}     # they are listed as "COMPONENTS"
inc/ )

catkin_package(CATKIN_DEPENDS pandubot_object_recognition move_base pandubot_face_detection)

add_executable(test_object_waypt_node src/test_object_waypt.cpp )
add_executable(test_executive_node  src/test_executive.cpp
                                    src/object_action_client.cpp 
                                    src/speech_synthesiser.cpp
                                    src/face_action_client.cpp
                                    src/navigation_action_client.cpp)

add_dependencies(test_object_waypt_node pandubot_object_recognition_generate_messages_cpp pandubot_face_detection_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_dependencies(test_executive_node pandubot_object_recognition_generate_messages_cpp pandubot_face_detection_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_object_waypt_node ${catkin_LIBRARIES}) 
target_link_libraries(test_executive_node ${catkin_LIBRARIES}) 

