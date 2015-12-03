cmake_minimum_required(VERSION 2.8.3)
project(pandubot_libraries)

set(YAML_LIBRARY /usr/lib/x86_64-linux-gnu/libyaml-cpp.so)
file(GLOB APRIL_TAG_LIB_SOURCE_FILES "src/apriltags/*.cc")

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        actionlib
                                        pandubot_object_recognition
                                        pandubot_face_detection
                                        sound_play
                                        cv_bridge
                                        geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS}
                    include/${PROJECT_NAME}/
                    /usr/include/eigen3)

catkin_package(INCLUDE_DIRS   include
               CATKIN_DEPENDS pandubot_object_recognition 
                              move_base
                              pandubot_face_detection
                              sound_play
                              cv_bridge
                              geometry_msgs
               LIBRARIES      pandubotActionClients
                              pandubotVoicePublisher
                              pandubotUtilities
                              pandubotSchemas
                              pandubotTypes)

add_library(pandubotActionClients src/object_action_client.cpp
                                  src/face_action_client.cpp
                                  src/navigation_action_client.cpp)
add_library(pandubotVoicePublisher src/voice_publisher.cpp)
add_library(pandubotUtilities src/utilities.cpp)
add_library(pandubotTypes src/utilities.cpp src/table_monitor.cpp)
add_library(pandubotSchemas src/perceptual_schema.cpp
                            src/motor_schema.cpp
                            ${APRIL_TAG_LIB_SOURCE_FILES})

add_dependencies(pandubotActionClients pandubot_object_recognition_generate_messages_cpp
                                       pandubot_face_detection_generate_message_cpp 
                                       ${catkin_EXPORTED_TARGETS})

target_link_libraries(pandubotActionClients ${catkin_LIBRARIES})
target_link_libraries(pandubotVoicePublisher ${catkin_LIBRARIES})
target_link_libraries(pandubotUtilities ${catkin_LIBRARIES})
target_link_libraries(pandubotSchemas ${catkin_LIBRARIES})
target_link_libraries(pandubotTypes ${catkin_LIBRARIES} ${YAML_LIBRARY} )

######################### Test executibles ##############################
add_executable(testPerceptualSchema test/testPerceptualSchema.cpp
                                    src/perceptual_schema.cpp)
target_link_libraries(testPerceptualSchema ${catkin_LIBRARIES})

add_executable(testMotorSchema test/testMotorSchema.cpp
                               src/motor_schema.cpp)
target_link_libraries(testMotorSchema ${catkin_LIBRARIES})

add_executable(testBehaviorSchema test/testBehaviorSchema.cpp
                                  src/utilities.cpp
                                  src/behavior_schema.cpp)
target_link_libraries(testBehaviorSchema ${catkin_LIBRARIES} ${YAML_LIBRARY})

add_executable(testTableMonitor test/test_TableMonitor.cpp
                                src/table_monitor.cpp
                                src/utilities.cpp)
target_link_libraries(testTableMonitor ${catkin_LIBRARIES} ${YAML_LIBRARY})



####### Optional Install Targets ################
# install(TARGETS pandubotActionClients
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        )
#        
# message(PATH ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


