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

# General ROS imports
import rospy

# ACS-specific imports
import ap_path_planning.follow_controller as fctrl


#---------------------------------------------
# Runs a node for a single follow_controller object
#---------------------------------------------

if __name__ == '__main__':

    ownAC = int(rospy.get_param("aircraft_id"))

    # Initialize ROS node
    rospy.init_node(fctrl.NODE_BASENAME)
    follower = fctrl.FollowController(fctrl.NODE_BASENAME, ownAC)
    follower.runAsNode(2.0, [], [ fctrl.TRKR_BASENAME, fctrl.CTRLR_BASENAME ], \
                                [ fctrl.AP_BASENAME, fctrl.CTRLR_BASENAME ])

