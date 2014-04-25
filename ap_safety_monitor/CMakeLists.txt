cmake_minimum_required(VERSION 2.8.3)
project(ap_safety_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Heartbeat.msg
)

add_service_files(
  FILES
  SetHealth.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

