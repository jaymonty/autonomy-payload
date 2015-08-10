cmake_minimum_required(VERSION 2.8.3)
project(ap_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  autopilot_bridge
  sensor_msgs
  nav_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  VehicleIntent.msg
  SwarmControlState.msg
  SwarmStateStamped.msg
  SwarmVehicleState.msg
  WaypointListStamped.msg
  FormationOrder.msg
  FormationOrderStamped.msg
  SwarmFormationOrder.msg
  SwarmFormationOrderStamped.msg
  ControllerState.msg
  ControllerGroupState.msg
  ControllerGroupStateStamped.msg
  MsgStat.msg
  MsgStatList.msg
  SwarmSearchWaypoint.msg
  SwarmSearchWaypointList.msg
  SwarmSearchOrder.msg
  SwarmSearchOrderStamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  autopilot_bridge
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime autopilot_bridge
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
