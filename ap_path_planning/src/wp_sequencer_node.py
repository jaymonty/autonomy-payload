#!/usr/bin/env python

#-----------------------------------------------------------------------
# Implements a ROS node with a single instance of WaypointSequencer
# Duane Davis, 2014
#
# Instantiates a single WaypointSequencer object, registers as a ROS
# node and executes a 1hz control loop 
#-----------------------------------------------------------------------


# Import ROS message and service types

# ACS-specific imports
import ap_path_planning.wp_sequencer as wpseq
from autopilot_bridge.msg import LLA

#---------------------------------------------
# Runs a node for a single wp_sequencer object
#---------------------------------------------
if __name__ == '__main__':
    # ROS and object initialization
    sequencer = wpseq.WaypointSequencer(wpseq.NODE_BASENAME, [])
    sequencer.runAsNode(10.0, [], [], [])

