#!/usr/bin/env python

import rosbag
import sys

if len(sys.argv) < 2:
    print "Please specify a bag file"
    sys.exit(1)

bag = rosbag.Bag(sys.argv[1])

last_sec = 0.0
count = 0

for topic, msg, t in bag.read_messages(topics=['/autopilot/heartbeat_ground']):
    sec = int(float(str(t)) / 1e9)
    if sec > last_sec:
        output = "%u,%u" % (last_sec, count)
        print output
        last_sec = sec
        count = 0

    count += 1

bag.close()

