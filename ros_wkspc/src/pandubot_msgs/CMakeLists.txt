cmake_minimum_required(VERSION 2.8.3)
project(pandubot_msgs)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        std_msgs
                                        message_generation
                                        move_base_msgs
                                        actionlib_msgs
                                        geometry_msgs)
add_message_files(FILES atomic_plan.msg
                        ReleaserSignal.msg
                        MotorSignalMoveBase.msg
                        MuxControl.msg
                        MuxMessage.msg
                        )
generate_messages(DEPENDENCIES std_msgs
                               move_base_msgs)
catkin_package(CATKIN_DEPENDS message_generation)



















































































































































































