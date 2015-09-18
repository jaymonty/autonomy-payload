#!/usr/bin/env python

'''
Quick way to run for a lot of logs, assuming this script and the logs are in
the same directory, using bash:

for i in *.bag; do echo "Processing $i ..."; ./rates.py $i > `basename -s .bag $i`.csv; done
'''

import rosbag
import sys

uavs = [5, 6, 7, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 22, 23, 24, 25, 26, 27,
        28, 29, 30, 31, 32, 33, 34, 36, 38, 39, 40, 41, 42, 43, 45, 48, 49, 50,
        51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 65, 66]

if len(sys.argv) < 2:
    print "Please specify a bag file"
    sys.exit(1)

bag = rosbag.Bag(sys.argv[1])

print "TIME," + ','.join(map(lambda x: str(x), uavs))

for topic, msg, t in bag.read_messages(topics=['/logging/recv_pose_rate']):
    cur_time = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9
    
    out = "%0.6f" % cur_time

    stats = {}
    for s in msg.stat:
        stats[s.id] = s

    for u in uavs:
        if u in stats:
            out += ",%u" % stats[u].count
        else:
            out += ",0"

    print out

