#!/usr/bin/env python

# Rudimentary follow-me code
# Listens to incoming network poses, downsamples, 
#   does some math, and sends coords to local autopilot
# Improved version written by Stefan Jorgensen, August 2014

import sys
import rospy
from argparse import ArgumentParser
from autopilot_bridge.msg import LLA
from ap_network_bridge.msg import NetOdometry

from nav_msgs.msg import Odometry
import math
from ap_lib import gps_utils


# These are set by command line arguments
 # Leader ID
apid = None
 #base altitude
BASE_ALT= None
 #altitude separation
ALT_SEP = None
 # Target following distance from leader
Rfollow = None #-10 #None #units meters
 # Overshoot distance for avoiding loiter 
Rovershoot = None #60 #units meters

# These are set by incoming ROS messages
tstamp = None
lat = None
lon = None
alt = None
self_lat = None
self_lon = None
self_alt = None

# Compute target waypoints with overshoot
def compute_follow_wp(lat2, lon2, lat1, lon1, self_lat, self_lon):
    global Rfollow, Rovershoot
    radius_of_earth = 6378100.0 #in meters
    radius_at_lat = radius_of_earth*math.cos((math.radians(lat1)+math.radians(lat2))/2)

  # Check for valid input
    if self_lat is None or self_lon is None:
        print "Missing self data"
        return None
    if lat2 is None or lon2 is None:
        print "Missing new leader data"
        return None
    if lat1 is None or lon1 is None:
        print "Missing prior leader data"
        return None

  # Project line back from leader:
    leader_bearing = gps_utils.gps_bearing(lat1,lon1,lat2,lon2)
    #Here we make an arclength assumption.
    a_lat = math.radians(lat2) - (Rfollow*math.cos(math.radians(leader_bearing)))/radius_of_earth 
    a_lon = math.radians(lon2) - (Rfollow*math.sin(math.radians(leader_bearing)))/radius_at_lat 

  # Project line forward:
    ab_bearing = gps_utils.gps_bearing(self_lat, self_lon, math.degrees(a_lat),math.degrees(a_lon))
    b_lat = math.degrees(a_lat + (Rovershoot*math.cos(math.radians(ab_bearing)))/radius_of_earth)
    b_lon = math.degrees(a_lon + (Rovershoot*math.sin(math.radians(ab_bearing)))/radius_at_lat)

    return (b_lat, b_lon)

# Handle incoming network pose messages
def pose_callback(data):
    global apid, tstamp, lat, lon, alt
    try:
        # Don't use poses from other than target aircraft
        if apid != data.sender_id:
            #print "Ignoring pose from wrong ID"
            return

        # Don't use poses not newer than last-heard
        t = float(data.odom.header.stamp.secs) + \
            float(data.odom.header.stamp.nsecs) / float(1e9)
        if tstamp is not None and t <= tstamp:
            print "Ignoring pose older than one received"
            return

        # Update last-heard state
        tstamp = t
        lat = data.odom.pose.pose.position.x
        lon = data.odom.pose.pose.position.y
        # DON'T USE UNLESS DOING ALTITUDE MATCHING
        alt = data.odom.pose.pose.position.z
    except Exception as ex:
        # If there's an error, null out data
        print "Net Pose Callback error: " + ex.args[0]
        tstamp = None  # Null so we don't get stuck in the future
        lat = None
        lon = None
        # DON'T USE UNLESS DOING ALTITUDE MATCHING
        alt = None


# Handle incoming gps odom messages
def odom_callback(data):
    global self_lat, self_lon, self_alt
    try:
        self_lat = data.pose.pose.position.x
        self_lon = data.pose.pose.position.y
        self_alt = data.pose.pose.position.z
    except Exception as ex:
        # If there's an error, null out data
        print "Odom Callback error: " + ex.args[0]
        self_lat = None
        self_lon = None
        self_alt = None

if __name__ == '__main__':
    # Parse command line arguments
    parser = ArgumentParser("rosrun ap_path_planning follow_me.py")
    parser.add_argument("--target", dest="aircraft", default=None,help="ID of aircraft to follow")
    parser.add_argument("--lookahead", dest="lookahead",default=-10,help="following distance to leader")
    parser.add_argument("--overshoot", dest="overshoot",default=40,help="Overshoot distance distance")
    parser.add_argument("--use-base-alt", dest="BASE_ALT", default=None,help="Use fixed altitude")
    parser.add_argument("--use-alt-sep", dest="ALT_SEP",default=None, help="Use altitude separation from leader")

    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])
    if args.BASE_ALT is not None:
        BASE_ALT = float(args.BASE_ALT)
        print "Using constant altitude %f" % (BASE_ALT)
    elif args.ALT_SEP is not None:
        ALT_SEP = float(args.ALT_SEP)
        print "Using altitude difference %f" % (ALT_SEP)
        if BASE_ALT is not None:
            print "Please use only --use-base-alt or --use-alt-sep options"
            sys.exit(1)
    else:
        print "Please supply either base altitude or separation altitude (--use-base-alt BASE_ALT or --use-alt-sep ALT_SEP"
        sys.exit(1)
    
    apid = int(args.aircraft)
    if apid is None: 
        print "Please supply target aircraft \(--target AIRCRAFT_ID\)"
        sys.exit(1) 
    Rfollow = float(args.lookahead) 
    Rovershoot = float(args.overshoot)

    # Initialize ROS node
    rospy.init_node("follow_me")

    # Listen to network pose messages
    sub_pose = rospy.Subscriber("/network/recv_pose",
                                NetOdometry,
                                pose_callback)

    # Listen to self pose messages
    sub_odom = rospy.Subscriber("/autopilot/acs_pose",
                                      Odometry,
                                      odom_callback)

    # Publish to autopilot's WP navigation
    pub_pl_wp = rospy.Publisher("/autopilot/payload_waypoint",LLA)

    print "\nFollowing aircraft %u ...\n" % (apid)

    #Containers for storing previous lat/lon data for leader
    last_lat = None
    last_lon = None

    # Loop at 1 Hz so we don't overwhelm the autopilot
    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        target_wp = None 
        if lat is not None and lon is not None:
            if last_lat is None or last_lon is None:
                # Need to initialize
                last_lat = lat
                last_lon = lon
            #elif self_lat is not None or self_lon is not None:
            else:
                target_wp = compute_follow_wp(lat, lon, last_lat, last_lon, self_lat, self_lon)
        # Only send if there is valid data to use
            if target_wp is not None and \
              tstamp is not None and alt is not None and self_alt is not None:
                target_lat, target_lon = target_wp
                # Build and send message
                lla = LLA()
                lla.lat = target_lat
                lla.lon = target_lon
                lla.alt = None
            # DON'T USE UNLESS DOING ALTITUDE MATCHING
                if BASE_ALT is not None:
                    lla.alt = BASE_ALT
                elif ALT_SEP is not None:
                    lla.alt = alt + ALT_SEP
                if lla.alt is not None: #verify valid altitude data
                    pub_pl_wp.publish(lla)
                    print "Sent to (%0.06f, %0.06f, %0.03f (leader alt: %0.03f))" % (lat, lon, lla.alt, alt)

            # Push old data to old containers
            last_lat = lat
            last_lon = lon

            # Null out old data so we don't resend the same point
            lat = None
            lon = None
            self_lat = None
            self_lon = None
            # DON'T USE UNLESS DOING ALTITUDE MATCHING
            alt = None

        # Don't check again for the rest of the second
        r.sleep()


