#!/usr/bin/env python

from pymavlink import mavutil
import sys
import time

try:
    if len(sys.argv) not in [3, 4]:
        raise Exception('')
    device = str(sys.argv[1])
    baudrate = int(sys.argv[2])
    max_time = 0
    if len(sys.argv) == 4:
        max_time = int(sys.argv[3])
except:
    print "usage: %s DEVICE BAUDRATE [MAX_TIME]" % sys.argv[0]
    sys.exit(1)

master = mavutil.mavlink_connection(device, baudrate, autoreconnect=True)

msg_types = {}
msg_count = 0
msg_bytes = 0

last_second = int(time.time())
if max_time > 0: max_time += last_second

while True:
    try:
        msg = master.recv_match(blocking=False)
    except:
        msg = None

    if msg:
        t = msg.get_type()
        if t not in msg_types: msg_types[t] = 0
        msg_types[t] += 1
        msg_count += 1
        msg_bytes += len(msg.get_msgbuf())
    else:
        time.sleep(0.1)

    t = int(time.time())
    if t > last_second:
        if msg_count > 0:
            util = 100.0 * (msg_bytes * 8) / (baudrate * 8 / 10)
            print "%u\tCOUNT: %u\tBYTES: %u\tUTIL: %3u%%" % \
                  (t, msg_count, msg_bytes, int(util))
            for k,v in sorted(msg_types.items()):
                print "  %-24s %u" % (k, v)

        last_second = t
        msg_count = 0
        msg_bytes = 0
        msg_types = {}

        time.sleep(0.1)

    if 0 < max_time < t:
        sys.exit(0)
