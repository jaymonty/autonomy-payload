#!/usr/bin/env python

#-----------------------------------------------------------------------
# Implements a ROS node with a single instance of SwarmTracker
# Duane Davis, 2014
#
# Instantiates a single SwarmTracker object, registers as a ROS
# node and executes a 10hz control loop 
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
    # Parse command line arguments
    parser = ArgumentParser("rosrun ap_perception swarm_tracker.py")
    parser.add_argument('-id', "--aircraft", dest="acft", \
                        help="ID (integer) of this aircraft", default=1)
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    subSwarmID = 0
    if args.acft == 1 and rospy.has_param("aircraft_id"):
        args.acft = rospy.get_param("aircraft_id")
    if rospy.has_param("subswarm_id"):
        subSwarmID = int(rospy.get_param("subswarm_id"))

    swarm_tracker = SwarmTracker(int(args.acft), subSwarmID, NODE_BASENAME)
    swarm_tracker.runAsNode(10.0, [], [ SELF_ODOM_BASENAME, \
                                        NET_BASENAME ], [])

