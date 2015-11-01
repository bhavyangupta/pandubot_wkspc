cmake_minimum_required(VERSION 2.8.3)
project(pandubot_stereo_vision)
install(DIRECTORY launch/
        DESTINATION  ${CATKIN_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)