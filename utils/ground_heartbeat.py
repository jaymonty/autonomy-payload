#!/usr/bin/env python

from optparse import OptionParser
import os
import sys
import time

# Load in ACS libraries
import ap_lib.acs_messages as messages
from ap_lib.acs_socket import Socket

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("ground_heartbeat.py [options]")
    parser.add_option("-d", "--device", dest="device", 
                      help="Network device to send on", default='')
    parser.add_option("-p", "--port", dest="port", type="int",
                      help="Network port to send to", default=5554)
    parser.add_option("--rate", dest="rate", type="int",
                      help="Rate at which to send (Hz)", default=2)
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
        sock = Socket(0xff, opts.port, opts.device, my_ip, bcast_ip, send_only=True)
    except Exception:
        print "Sorry, couldn't start up the sending socket"
        sys.exit(1)
    
    message = messages.Heartbeat()
    message.msg_dst = Socket.ID_BCAST_ALL
    message.msg_secs = 0
    message.msg_nsecs = 0
    message.counter = 0

    while True:
        sock.send(message)
        print "Heartbeat sent (%u)" % message.counter

        message.counter += 1

        time.sleep(1.0 / float(opts.rate))

