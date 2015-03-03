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

        # Counts
        self.total = 0
        self.period = 0
        self.resets = 0

        # Rolling average
        self.roll_max = rolling
        self.roll_bins = [0]

        # Last time seen
        self.last = 0

    def increment(self, t=time.time()):
        self.total += 1
        self.period += 1
        self.roll_bins[-1] += 1
        self.last = t

    def reset(self):
        self.period = 0
        self.resets += 1
        self.roll_bins.append(0)
        extra_bins = len(self.roll_bins) - self.roll_max
        if extra_bins > 0:
            self.roll_bins = self.roll_bins[extra_bins:]

    def report(self, t=time.time()):
        return "%2X: %1u/%3.1f/%3.1f/%5u/%4.1f" % \
               (self.msg_type,
                self.period,
                sum(self.roll_bins) / float(len(self.roll_bins)),
                float(self.total) / float(self.resets + 1),
                self.total,
                t - self.last)

class MsgStats(object):
    def __init__(self, acid, rolling=10):
        self.acid = acid
        self.roll_max = rolling
        self.last = 0
        self.msgtypes = {}

    def update(self, msg_type, t=time.time()):
        self.last = t
        if msg_type not in self.msgtypes:
            self.msgtypes[msg_type] = MsgTypeStat(msg_type, self.roll_max)
        self.msgtypes[msg_type].increment(t)

    def reset(self):
        for mt in self.msgtypes:
            self.msgtypes[mt].reset()

    def report(self, t=time.time()):
        line = "ID %3u" % self.acid
        for mt in self.msgtypes:
            line += "\t%s" % self.msgtypes[mt].report(t)
        return line

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("net_parser.py [options]")
    parser.add_option("-d", "--device", dest="device", 
                      help="Network device to listen on", default='')
    parser.add_option("-p", "--port", dest="port", type="int",
                      help="Network port to listen on", default=5554)
    parser.add_option("--rolling", dest="rolling", type="int",
                      help="Size of rolling average", default=10)
    parser.add_option("--offline", dest="offline", type="int",
                      help="Time before considering offline", default=30)
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
    last_sec = 0
    
    while True:
        # If we've rolled over seconds, display and reset
        t = time.time()
        if int(t) > last_sec:
            last_sec = int(t)

            # Only keep those that are still "online" (reporting)
            stats = {k:v for k,v in stats.items() if v.last + opts.offline >= t}

            # Refresh screen
            os.system('clear')
            print "ID xxx  TYPE: CUR/ROL(%u)/AVG/TOT/LST(%u)  ...\n" % \
                  (opts.rolling, opts.offline)
            for s in sorted(stats):
                print stats[s].report(t)
                stats[s].reset()
            sys.stdout.flush()

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
        stats[msg.msg_src].update(msg.msg_type, t)

