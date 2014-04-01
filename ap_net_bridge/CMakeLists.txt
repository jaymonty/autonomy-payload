cmake_minimum_required(VERSION 2.8.3)
project(ap_network_bridge)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
)

add_message_files(
  FILES
  shared_lla.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs 
  sensor_msgs 
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
