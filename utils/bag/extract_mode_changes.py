#!/usr/bin/env python

import rosbag
import sys

if len(sys.argv) < 2:
    print "Please specify a bag file"
    sys.exit(1)

bag = rosbag.Bag(sys.argv[1])

act = None
for topic, msg, t in bag.read_messages(topics=['/controllers/selector_status', '/controllers/follower_set']):
    if topic == '/controllers/follower_set':
        print "FOLLOW %d" % msg.order.leader_id
        continue

    if msg.state.active_controller == act:
        continue
    tim = float(str(t)) / 1e9
    act = msg.state.active_controller
    print "%0.03f\t%d" % (tim, act)

bag.close()
