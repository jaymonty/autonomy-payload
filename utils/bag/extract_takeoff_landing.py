#!/usr/bin/env python

import rosbag
import sys
import time

def printer(event, t):
    t_loc = time.localtime(t)
    t_str = time.strftime('%Y-%m-%d %H:%M:%S', t_loc)
    print "%s: %s (%d)" % (event, t_str, t)

if len(sys.argv) < 2:
    print "Please specify a bag file"
    sys.exit(1)

bag = rosbag.Bag(sys.argv[1])

TO_THRESH = 30.0
LD_THRESH = 20.0
old_alt = 0.0

for topic, msg, t in bag.read_messages(topics=['/autopilot/status']):
    tim = float(str(t)) / 1e9
    alt = msg.alt_rel / 1e3

    if old_alt < TO_THRESH < alt:
        printer('Takeoff', tim)
    elif old_alt > LD_THRESH > alt:
        printer('Landing', tim)

    old_alt = alt

bag.close()
