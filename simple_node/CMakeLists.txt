cmake_minimum_required(VERSION 2.8.3)
project(simple_node)

find_package(catkin REQUIRED COMPONENTS 
             genmsg
             moveit_ros_planning_interface
             roscpp
             std_msgs
             tf_conversions
             visualization_msgs)

add_service_files(FILES SendOffset.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(simple_node src/simple_node.cpp)
target_link_libraries(simple_node ${catkin_LIBRARIES})
