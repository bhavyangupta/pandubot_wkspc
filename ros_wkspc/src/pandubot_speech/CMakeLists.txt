cmake_minimum_required(VERSION 2.4.6)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(pandubot_speech)
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
message_generation
)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)



#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_message_files(
FILES
pandu_voice_message.msg
)

generate_messages(
DEPENDENCIES
std_msgs
)


catkin_package(
  # INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rospy 
   CATKIN_DEPENDS message_runtime
   DEPENDS pocketsphinx sound_play std_msgs
   )

#rosbuild_init()

#include_directories(
#	${catkin_INCLUDE_DIRS}
#)

#add_executable(pandubot_voice src/pandubot_voice.py)
#target_link_libraries(pandubot_voice ${catkin_LIBRARIES})


#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
