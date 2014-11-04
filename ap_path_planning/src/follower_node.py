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

# ACS-specific imports
import ap_path_planning.follow_controller as fctrl


#---------------------------------------------
# Runs a node for a single follow_controller object
#---------------------------------------------

if __name__ == '__main__':
    # Parse command line arguments
    parser = ArgumentParser("rosrun ap_path_planning follow_me.py")
    parser.add_argument("--id", dest="own_aircraft", default=None, help="ID of this aircraft")
    parser.add_argument("--target", dest="follow_aircraft", default=None, help="ID of aircraft to follow")
    parser.add_argument("--distance", dest="distance", default=fctrl.FOLLOW_DISTANCE, help="following distance from leader")
    parser.add_argument("--offset", dest="offset", default=0.0, help="Clockwise offset from directly astern")
    parser.add_argument("--overshoot", dest="overshoot", default=fctrl.OVERSHOOT, help="Overshoot distance")
    parser.add_argument("--use-base-alt", dest="BASE_ALT", default=None, help="Use fixed altitude")
    parser.add_argument("--use-alt-sep", dest="ALT_SEP", default=None, help="Use altitude separation from leader")

    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])
    alt = None
    if args.BASE_ALT is not None:
        alt = float(args.BASE_ALT)
        altMode = fctrl.BASE_ALT_MODE
        print "Using constant altitude " + str(args.BASE_ALT)
    elif args.ALT_SEP is not None:
        alt = float(args.ALT_SEP)
        altMode = fctrl.ALT_SEP_MODE
        print "Using altitude difference " + str(args.ALT_SEP)
        if BASE_ALT is not None:
            print "Please use only --use-base-alt or --use-alt-sep options"
            sys.exit(1)
    else:
        print "Please supply either base altitude or separation altitude (--use-base-alt BASE_ALT or --use-alt-sep ALT_SEP)"
        sys.exit(1)

    if args.follow_aircraft is None: 
        print "Please supply target aircraft \(--target AIRCRAFT_ID\)"
        sys.exit(1) 
    followID = int(args.follow_aircraft)

    if args.own_aircraft == None and rospy.has_param("aircraft_id"):
        args.own_aircraft = rospy.get_param("aircraft_id")
    if args.own_aircraft is None:
        print "Please supply own aircraft ID \(--id AIRCRAFT_ID\)"
        sys.exit(1)
    ownAC = int(args.own_aircraft)

    if followID == ownAC:
        print "Attempt to set leader and follower to the same ID"
        sys.exit(1)

    distance = float(args.distance)
    offset = float(args.offset)
    overshoot = float(args.overshoot)

    # Initialize ROS node
    rospy.init_node(fctrl.NODE_BASENAME)
    print "\nFollowing aircraft " + str(followID) + " ...\n"

    follower = fctrl.FollowController(fctrl.NODE_BASENAME, ownAC, followID, distance, \
                                      offset, overshoot, altMode, alt)
    follower.set_active(True)
    follower.runAsNode(2.0, [], [ fctrl.TRKR_BASENAME, fctrl.CTRLR_BASENAME ], [ fctrl.AP_BASENAME ])

