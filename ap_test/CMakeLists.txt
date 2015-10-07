cmake_minimum_required(VERSION 2.8.3)
project(ap_test)

find_package(catkin REQUIRED COMPONENTS
    rospy
    rostest
)

catkin_package()

#These tests can be run when "catkin_make test" is typed:
#Currently can only run library tests via catkin make -- can't startup
#ROS nodes during catkin_make at the moment.  Test separately using rostest.
#FILE(GLOB ROS_unit "ROS_unit/*.py")
#catkin_add_nosetests(${ROS_unit})
