#!/usr/bin/env python

#-----------------------------------------------------------------------
# Implements a ROS node with a single instance of SwarmSubscriber
# Duane Davis, 2014
#
# Instantiates a single SwarmSubscriber object, registers as a ROS
# node and subscribes to the swarm topic (just prints received info)
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
        print "\ntime: " + str(swarm_sub.timestamp)
        for vid in swarm_sub.swarm:
            print "aicraft_id: " + str(vid)
            print str(swarm_sub.swarm[vid].state)
        r.sleep()

