cmake_minimum_required(VERSION 2.8.3)
project(pandubot_types)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS}
                    include/${PROJECT_NAME}/)

catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS geometry_msgs
               LIBRARIES pandubotTypes)

add_library(pandubotTypes src/TableMonitor.cpp)

target_link_libraries(pandubotTypes ${catkin_LIBRARIES})

