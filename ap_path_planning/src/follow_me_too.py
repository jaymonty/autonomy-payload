#!/usr/bin/env python

# Rudimentary follow-me code
# Listens to incoming network poses, downsamples, 
#   does some math, and sends coords to local autopilot
# Improved version written by Stefan Jorgensen, August 2014

import sys
import rospy
from argparse import ArgumentParser
from autopilot_bridge.msg import LLA
from ap_network_bridge.msg import NetPoseStamped
from nav_msgs.msg import Odometry
import math

# These are set by command line arguments
 # Leader ID
apid = None
 #base altitude
BASE_ALT= None
 #altitude separation
ALT_SEP = None
 # Target following distance from leader
Rfollow = -10 #None #units meters
 # Overshoot distance for avoiding loiter 
Rovershoot = 60 #units meters

# These are set by incoming ROS messages
tstamp = None
lat = None
lon = None
alt = None
self_lat = None
self_lon = None
self_alt = None

# Return small-angle approximation based x,y distance and angle relative to E
def gps_distance2(lat1, lon1, lat2, lon2, isRadians):
    radius_of_earth = 6378100.0 #in meters
    if isRadians == False:
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        lon1 = math.radians(lon1)
        lon2 = math.radians(lon2)
 # Compute radius of earth at longitude
    radius_at_lat = radius_of_earth*math.cos((lat1+lat2)/2)
 # We can use these as distance since sin(x) ~ x for x << 1
    dlat = math.sin(lat2 - lat1)*radius_of_earth
    dlon = math.sin(lon2 - lon1)*radius_at_lat
 # Compute heading relative to E (for convenience, sorry for being non-standard)
    angle = math.atan2(dlat,dlon)
    return (dlat, dlon, angle)

# Compute target waypoints with overshoot
def compute_follow_wp(lat, lon, lat_last, lon_last, self_lat, self_lon):
    global Rfollow, Rovershoot
  # Check for valid input
    if self_lat is None or self_lon is None:
        print "Missing self data"
        return None
    if lat is None or lon is None:
        print "Missing new leader data"
        return None
    if lat_last is None or lon_last is None:
        print "Missing prior leader data"
        return None
  #load current/past leader lat/lon into containers 
    radius_of_earth = 6378100.0 #in meters
    lat1 = math.radians(lat_last)
    lon1 = math.radians(lon_last)
    lat2 = math.radians(lat)
    lon2 = math.radians(lon)
    self_latrad = math.radians(self_lat)
    self_lonrad = math.radians(self_lon)
    radius_at_lat = radius_of_earth*math.cos((lat1+lat2)/2)
    leader_dlat, leader_dlon, leader_angle = gps_distance2(lat1, lon1, lat2, lon2, True)
  #Project line back:
    a_lat = lat2 - (Rfollow*math.sin(leader_angle))/radius_of_earth #assumes arclength ~ straight line
    a_lon = lon2 - (Rfollow*math.cos(leader_angle))/radius_at_lat #same assumption.
  #Project line forward:
    a_dlat, a_dlon, a_angle = gps_distance2(self_latrad, self_lonrad, a_lat, a_lon, True)
    b_lat = math.degrees(a_lat + (Rovershoot*math.sin(a_angle))/radius_of_earth)
    b_lon = math.degrees(a_lon + (Rovershoot*math.cos(a_angle))/radius_at_lat)

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
        alt = data.pose.pose.position.z
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
    parser.add_argument("aircraft", help="ID of aircraft to follow")
    #parser.add_argument("Rfollow", help="following distance")
    parser.add_argument("--use-base-alt", dest="BASE_ALT", default=None)
    parser.add_argument("--use-alt-sep", dest="ALT_SEP",default=None)
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
    # Rfollow = float(args.Rfollow) 
    # Initialize ROS node
    rospy.init_node("follow_me")

    # Listen to network pose messages
    sub_pose = rospy.Subscriber("/network/recv_pose",
                                NetPoseStamped,
                                pose_callback)

    # Listen to self pose messages
    sub_odom = rospy.Subscriber("/autopilot/gps_odom",
                                      Odometry,
                                      odom_callback)

    # Publish to autopilot's WP navigation
    pub_pl_wp = rospy.Publisher("/autopilot/payload_waypoint",
                                LLA)

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


