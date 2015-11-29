cmake_minimum_required(VERSION 2.8.3)
project(pandubot_libraries)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        actionlib
                                        pandubot_object_recognition
                                        pandubot_face_detection
                                        sound_play)

include_directories(${catkin_INCLUDE_DIRS}
                    include/${PROJECT_NAME}/)

catkin_package(INCLUDE_DIRS   include
               CATKIN_DEPENDS pandubot_object_recognition 
                              move_base
                              pandubot_face_detection
                              sound_play
               LIBRARIES      pandubotActionClients
                              pandubotVoicePublisher
                              pandubotUtilities)

add_library(pandubotActionClients src/object_action_client.cpp
                                  src/face_action_client.cpp
                                  src/navigation_action_client.cpp)

add_library(pandubotVoicePublisher src/voice_publisher.cpp)

add_library(pandubotUtilities src/utilities.cpp)

add_dependencies(pandubotActionClients pandubot_object_recognition_generate_messages_cpp
                                       pandubot_face_detection_generate_message_cpp 
                                       ${catkin_EXPORTED_TARGETS})

target_link_libraries(pandubotActionClients ${catkin_LIBRARIES})

target_link_libraries(pandubotVoicePublisher ${catkin_LIBRARIES})

target_link_libraries(pandubotUtilities ${catkin_LIBRARIES})

####### Optional Install Targets ################
# install(TARGETS pandubotActionClients
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        )
#        
# message(PATH ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


