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
import swarm_tracker as tracker


if __name__ == '__main__':
    # Parse command line arguments
    parser = ArgumentParser("rosrun ap_perception swarm_tracker.py")
    parser.add_argument('-n', "--nodename", dest="nodename", \
                        help="Name for the ROS node to register as", \
                        default=tracker.NODE_BASENAME)
    parser.add_argument('-id', "--aircraft", dest="acft", \
                        help="ID (integer) of this aircraft", default=1)
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])
    
    # Initialize ROS node & instantiate a SwarmTracker object
    rospy.init_node("swarm_tracker")
    swarm_tracker = tracker.SwarmTracker(int(args.acft))
    swarm_tracker.runAsNode(10.0, [], [ tracker.SELF_ODOM_BASENAME, \
                                        tracker.NET_ODOM_BASENAME ], [])

