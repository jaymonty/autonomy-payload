#!/usr/bin/env python

#-----------------------------------------------------------------------
# Implements a ROS node with a single instance of WaypointSequencer
# Duane Davis, 2014
#
# Instantiates a single WaypointSequencer object, registers as a ROS
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
import ap_path_planning.wp_sequencer as wpseq


#---------------------------------------------
# Runs a node for a single wp_sequencer object
#---------------------------------------------
if __name__ == '__main__':
    parser = ArgumentParser("rosrun ap_path_planning wp_sequencer_node.py")
    parser.add_argument('-n', "--nodename", dest="nodename", \
                        help="Name for the ROS node to register as", \
                        default=wpseq.NODE_BASENAME)
    parser.add_argument('-o', "--odombase", dest="odombase", \
                        help="Basename for the ROS odometry topic", \
                        default=wpseq.ODOM_BASENAME)
    parser.add_argument('-a', "--apbase", dest="apbase", \
                        help="Basename for the ROS autopilot topics", \
                        default=wpseq.AP_BASENAME)
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    # ROS and object initialization
    sequencer = wpseq.WaypointSequencer(args.nodename, [])
    sequencer.setSequence([ [35.718299, -120.766609, 100.0], \
                            [35.718574, -120.762931, 100.0], \
                            [35.719459, -120.765800, 100.0], \
                            [35.717185, -120.763793, 100.0], \
                            [35.718330, -120.764748, 100.0] ])
    sequencer.runAsNode(10.0, [], [ args.odombase ], [ args.apbase ])

