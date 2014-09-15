#!/usr/bin/env python

#-----------------------------------------------------------------------
# Implements a ROS node with a single instance of SwarmTracker
# Duane Davis, 2014
#
# Instantiates a single SwarmTracker object, registers as a ROS
# node and executes a 1hz control loop 
#-----------------------------------------------------------------------

# Import a bunch of libraries
import sys

# Standard Python imports
from argparse import ArgumentParser

# General ROS imports
import rospy

# Import ROS message and service types

# ACS-specific imports
from ap_perception.swarm_tracker import *


if __name__ == '__main__':
    rospy.init_node("swarm_subscriber")
    swarm_sub = SwarmTrackerSubscriber()
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        print "time: " + str(swarm_sub.timestamp)
        for vid in swarm_sub.swarm:
            vehicle = swarm_sub.swarm[vid]
            print "vehicleID: " + str(vid)
            print "pose:"
            print "  position:"
            print "    x: " + str(vehicle.pose.pose.position.x)
            print "    y: " + str(vehicle.pose.pose.position.y)
            print "    z: " + str(vehicle.pose.pose.position.z)
            print "  orientation:"
            print "    x: " + str(vehicle.pose.pose.orientation.x)
            print "    y: " + str(vehicle.pose.pose.orientation.y)
            print "    z: " + str(vehicle.pose.pose.orientation.z)
            print "    w: " + str(vehicle.pose.pose.orientation.w)
            print "  covariance: " + str(vehicle.pose.covariance)
            print "velocity: "
            print "  twist:"
            print "    linear:"
            print "      x: " + str(vehicle.velocity.twist.linear.x)
            print "      y: " + str(vehicle.velocity.twist.linear.y)
            print "      z: " + str(vehicle.velocity.twist.linear.z)
            print "    angular:"
            print "      x: " + str(vehicle.velocity.twist.angular.x)
            print "      y: " + str(vehicle.velocity.twist.angular.y)
            print "      z: " + str(vehicle.velocity.twist.angular.z)
            print "  covariance: " + str(vehicle.velocity.covariance)

        r.sleep()

