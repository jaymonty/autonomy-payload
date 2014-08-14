#!/usr/bin/env python

# Rudimentary follow-me code
# Listens to incoming network poses, downsamples, and
#   sends coords to local autopilot
# Stupid version written by Mike Clement, August 2014

import sys
import rospy
from argparse import ArgumentParser
from autopilot_bridge.msg import LLA
from ap_network_bridge.msg import NetPoseStamped

# These are set by command line arguments
apid = None
alt = None

# These are set by incoming ROS messages
tstamp = None
lat = None
lon = None

# Handle incoming network pose messages
def pose_callback(data):
    global apid, tstamp, lat, lon, alt
    try:
        # Don't use poses from other than target aircraft
        if apid != data.sender_id:
            print "Ignoring pose from wrong ID"
            return

        # Don't use poses not newer than last-heard
        t = float(data.pose.header.stamp.secs) + \
            float(data.pose.header.stamp.nsecs) / float(1e9)
        if tstamp is not None and t <= tstamp:
            print "Ignoring pose older than one received"
            return

        # Update last-heard state
        tstamp = t
        lat = data.pose.pose.position.x
        lon = data.pose.pose.position.y

        # DON'T USE UNLESS DOING ALTITUDE MATCHING
        #alt = data.pose.pose.position.z
    except Exception as ex:
        # If there's an error, null out data
        print "Callback error: " + ex.args[0]
        tstamp = None  # Null so we don't get stuck in the future
        lat = None
        lon = None

        # DON'T USE UNLESS DOING ALTITUDE MATCHING
        #alt = None

if __name__ == '__main__':
    # Parse command line arguments
    parser = ArgumentParser("rosrun ap_path_planning follow_me.py")
    parser.add_argument("aircraft", help="ID of aircraft to follow")
    parser.add_argument("alt", help="altitude to hold")
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])
    apid = int(args.aircraft)
    alt = float(args.alt)

    # Initialize ROS node
    rospy.init_node("follow_me")

    # Listen to network pose messages
    sub_pose = rospy.Subscriber("/network/recv_pose",
                                NetPoseStamped,
                                pose_callback)

    # Publish to autopilot's WP navigation
    pub_pl_wp = rospy.Publisher("/autopilot/payload_waypoint",
                                LLA)

    print "\nFollowing aircraft %u at altitude %u ...\n" % (apid, alt)

    # Loop at 1 Hz so we don't overwhelm the autopilot
    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        # Only send if there is valid data to use
        if tstamp is not None and \
           lat is not None and \
           lon is not None and \
           alt is not None:

            # Build and send message
            lla = LLA()
            lla.lat = lat
            lla.lon = lon
            lla.alt = alt
            pub_pl_wp.publish(lla)
            print "Sent to (%0.06f, %0.06f, %0.03f)" % (lat, lon, alt)

            # Null out old data so we don't resend the same point
            lat = None
            lon = None

            # DON'T USE UNLESS DOING ALTITUDE MATCHING
            #alt = None

        # Don't check again for the rest of the second
        r.sleep()


