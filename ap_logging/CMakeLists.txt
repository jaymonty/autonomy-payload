cmake_minimum_required(VERSION 2.8.3)
project(ap_logging)

find_package(catkin REQUIRED COMPONENTS
    rospy
    ap_network_bridge
    ap_autopilot_bridge
)

#add_message_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES 
#)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
