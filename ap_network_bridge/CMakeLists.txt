cmake_minimum_required(VERSION 2.8.3)
project(ap_network_bridge)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    sensor_msgs
    nav_msgs
    message_generation
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
