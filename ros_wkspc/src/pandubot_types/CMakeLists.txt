cmake_minimum_required(VERSION 2.8.3)
project(pandubot_types)

set(YAML_LIBRARY /usr/lib/x86_64-linux-gnu/libyaml-cpp.so)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        geometry_msgs
                                        pandubot_libraries)

include_directories(${catkin_INCLUDE_DIRS}
                    include/${PROJECT_NAME}/)

catkin_package(INCLUDE_DIRS   include
               CATKIN_DEPENDS geometry_msgs
                              pandubot_libraries
               LIBRARIES      pandubotTypes)

add_library(pandubotTypes src/table_monitor.cpp)
target_link_libraries(pandubotTypes ${catkin_LIBRARIES} ${YAML_LIBRARY})

add_executable(test_TableMonitor_node test/test_TableMonitor.cpp 
                                      src/table_monitor.cpp
                                      )

target_link_libraries(test_TableMonitor_node ${catkin_LIBRARIES} ${YAML_LIBRARY})
