cmake_minimum_required(VERSION 2.8.3)
project(pandubot_executive)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        pandubot_libraries)

include_directories(${catkin_INCLUDE_DIRS}  
                    ${pandubot_libraries_INCLUDE_DIRS}
                    inc/)

catkin_package(CATKIN_DEPENDS pandubot_libraries)

# add_executable(test_object_waypt_node src/test_object_waypt.cpp )
add_executable(test_executive_node  src/test_executive.cpp
                                    src/speech_synthesiser.cpp)

# add_dependencies(test_object_waypt_node pandubot_libraries ${catkin_EXPORTED_TARGETS})

add_dependencies(test_executive_node pandubot_libraries ${catkin_EXPORTED_TARGETS})

# target_link_libraries(test_object_waypt_node ${catkin_LIBRARIES}) 
target_link_libraries(test_executive_node ${catkin_LIBRARIES}) 

