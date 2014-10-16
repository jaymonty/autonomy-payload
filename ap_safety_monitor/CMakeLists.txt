cmake_minimum_required(VERSION 2.8.3)
project(ap_safety_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

