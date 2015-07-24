#!/usr/bin/env python

import rosbag
import sys

if len(sys.argv) < 2:
    print "Please specify a bag file"
    sys.exit(1)

bag = rosbag.Bag(sys.argv[1])

for topic, msg, t in bag.read_messages(topics=['/autopilot/acs_pose']):
    tim = float(str(t)) / 1e9
    lat = msg.pose.pose.position.lat
    lon = msg.pose.pose.position.lon
    alt = msg.pose.pose.position.alt
    print "%0.06f\t%0.07f\t%0.07f\t%0.02f" % (tim, lat, lon, alt)

bag.close()
