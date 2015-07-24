#!/usr/bin/env python

import rosbag
import sys

if len(sys.argv) < 2:
    print "Please specify a bag file"
    sys.exit(1)

bag = rosbag.Bag(sys.argv[1])

clu = []  # Counts-Look-Up to keep columns in order
counts = {}  # Actual counts of messages, per ID, reset each second

last_sec = 0.0

for topic, msg, t in bag.read_messages(topics=['/network/recv_pose']):
    sec = int(float(str(t)) / 1e9)
    if sec > last_sec:
        output = "%d\t" % last_sec
        for i in clu:
            output += "%d\t" % counts[i]
            counts[i] = 0
        print output
        last_sec = sec
            
    if msg.vehicle_id not in clu:
        clu.append(msg.vehicle_id)
        counts[msg.vehicle_id] = 0
    counts[msg.vehicle_id] += 1

output = "IDs:\t"
for i in clu:
    output += "\t%d" % i
print output

bag.close()

