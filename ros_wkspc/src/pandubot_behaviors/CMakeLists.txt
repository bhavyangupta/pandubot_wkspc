cmake_minimum_required(VERSION 2.8.3)
project(pandubot_behaviors)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        move_base_msgs
                                        actionlib
                                        pandubot_msgs
                                        std_msgs
                                        april_tag
                                        pandubot_libraries
                                        pandubot_msgs
                                        actionlib_msgs
                                        )
include_directories(${catkin_INCLUDE_DIRS}
                    include/${PROJECT_NAME})

catkin_package(INCLUDE_DIRS   include
               CATKIN_DEPENDS move_base_msgs
                              actionlib
                              pandubot_msgs
                              std_msgs
                              april_tag
                              pandubot_libraries
                              pandubot_msgs
                              actionlib_msgs              
                              )

################################# Main Nodes ################################
add_executable(commandMuxNode src/mux_node.cpp
                              src/command_mux.cpp)
target_link_libraries(commandMuxNode ${catkin_LIBRARIES})

add_executable(wayptBehaviorNode src/waypoint_behavior_node.cpp
                                  src/pandubot_behaviors.cpp
                                  src/pandubot_subscribers.cpp
                                  src/pandubot_publishers.cpp)
target_link_libraries(wayptBehaviorNode ${catkin_LIBRARIES})

add_executable(objectReportBehaviorNode src/object_report_behavior_node.cpp
                                        src/pandubot_behaviors.cpp
                                        src/pandubot_subscribers.cpp
                                        src/pandubot_publishers.cpp)
target_link_libraries(objectReportBehaviorNode ${catkin_LIBRARIES})

add_executable(finalNode src/main.cpp
                         src/pandubot_behaviors.cpp
                         src/pandubot_subscribers.cpp
                         src/pandubot_publishers.cpp)
target_link_libraries(finalNode ${catkin_LIBRARIES})



################################# Test Nodes ###############################
# add_executable(testMoveBaseMuxNode test/testMoveBaseMultiplexing.cpp
#                                    src/command_mux.cpp)
# add_dependencies(testMoveBaseMuxNode pandubot_msgs_generate_messages_cpp 
#                 ${catkin_EXPORTED_TARGETS})
# target_link_libraries(testMoveBaseMuxNode ${catkin_LIBRARIES})


add_executable(testPandubotSubscribersNode test/testPandubotSubscribers.cpp
                                           src/pandubot_subscribers.cpp
                                           )
add_dependencies(testPandubotSubscribersNode april_tag_generate_messges_cpp 
                 ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(testPandubotSubscribersNode ${catkin_LIBRARIES})

add_executable(testPandubotBehaviorsNode test/testPandubotBehaviors.cpp 
                                         src/pandubot_behaviors.cpp
                                         src/pandubot_subscribers.cpp
                                         src/pandubot_publishers.cpp)
target_link_libraries(testPandubotBehaviorsNode ${catkin_LIBRARIES})



### Nodes to test mux prototype ###
# add_executable(testMuxNode test/mux_test/mux.cpp)
# add_executable(testTagSenderNode test/mux_test/april_tag_sender.cpp)
# add_executable(testObjectSenderNode test/mux_test/object_sender.cpp)
# add_executable(testWayptSenderNode test/mux_test/waypt_sender.cpp)
# 
# target_link_libraries(testMuxNode ${catkin_LIBRARIES})
# target_link_libraries(testTagSenderNode ${catkin_LIBRARIES})
# target_link_libraries(testObjectSenderNode ${catkin_LIBRARIES})
# target_link_libraries(testWayptSenderNode ${catkin_LIBRARIES})
