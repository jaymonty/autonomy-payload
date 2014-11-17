#!/usr/bin/env python

from optparse import OptionParser
import os
import sys
import time

# Load in ACS libraries
import ap_lib.acs_messages as messages
from ap_lib.acs_socket import Socket

class MsgTypeStat(object):
    def __init__(self, msg_type, rolling=10):
        self.msg_type = msg_type
        self.total = 0
        self.period = 0
        self.resets = 0

        # rolling average
        self.rolling = rolling
        self.past_periods = []

    def increment(self):
        self.total += 1
        self.period += 1

    def reset(self):
        self.past_periods.append(self.period)
        while len(self.past_periods) > self.rolling:
            self.past_periods = self.past_periods[1:]
        self.period = 0
        self.resets += 1

    def __str__(self):
        rol_cnt = min(self.rolling, len(self.past_periods))
        rol_avg = 'NaN'
        if rol_cnt > 0:
            rol_avg = "%1.1f" % \
                      (sum(self.past_periods) / float(rol_cnt))

        return "%2X: %1u/%s/%1.1f/%5u" % \
               (self.msg_type,
                self.period,
                rol_avg,
                float(self.total) / float(self.resets + 1),
                self.total)

class MsgStats(object):
    def __init__(self, acid, rolling=10):
        self.acid = acid
        self.rolling = rolling
        self.msgtypes = {}

    def update(self, msg_type):
        if msg_type not in self.msgtypes:
            self.msgtypes[msg_type] = MsgTypeStat(msg_type, self.rolling)
        self.msgtypes[msg_type].increment()

    def reset(self):
        for t in self.msgtypes:
            self.msgtypes[t].reset()

    def __str__(self):
        line = "ID %3u" % self.acid
        for t in self.msgtypes:
            line += "\t%s" % self.msgtypes[t]
        return line

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("net_parser.py [options]")
    parser.add_option("--device", dest="device", 
                      help="Network device to listen on", default='')
    parser.add_option("--port", dest="port", type="int",
                      help="Network port to listen on", default=5554)
    parser.add_option("--rolling", dest="rolling", type="int",
                      help="Size of rolling average", default=10)
    parser.add_option("--lo-reverse", dest="lo_reverse",
                      action="store_true", default=False,
                      help="If using lo, reverse the addresses")
    (opts, args) = parser.parse_args()
    
    # NOTE: This is a hack to work with SITL
    my_ip = None
    bcast_ip = None
    if opts.device == 'lo':
        my_ip = '127.0.1.1'
        bcast_ip = '127.0.0.1'
        if opts.lo_reverse:
            (my_ip, bcast_ip) = (bcast_ip, my_ip)
    
    try:
        sock = Socket(0xff, opts.port, opts.device, my_ip, bcast_ip)
    except Exception:
        print "Sorry, couldn't start up the listening socket"
        sys.exit(1)

    # Dictionary of aircraft IDs
    stats = {}

    # Track current second we're in
    last_time = 0
    
    while True:
        # If we've rolled over seconds, display and reset
        t = int(time.time())
        if t > last_time:
            os.system('clear')
            print "ID xxx  TYPE: CUR/ROL(%u)/AVG/TOT  ...\n" % opts.rolling
            for s in sorted(stats):
                print stats[s]
                stats[s].reset()

            sys.stdout.flush()
            last_time = t

        # Look for new messages
        msg = sock.recv()
        if msg == None:  # No packets to receive, wait a bit
            time.sleep(0.1)
            continue
        elif msg == False:  # Packet received, but ignored
            continue

        # Update stats
        if msg.msg_src not in stats:
            stats[msg.msg_src] = MsgStats(msg.msg_src, opts.rolling)
        stats[msg.msg_src].update(msg.msg_type)

