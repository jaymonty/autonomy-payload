#!/usr/bin/env python

from optparse import OptionParser
import os
import sys
import time

# Load in ACS libraries
import acs.acs_messages as acs_messages
from acs.acs_socket import Socket

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("ground_heartbeat.py [options]")
    parser.add_option("--device", dest="device", 
                      help="Network device to send on", default='')
    parser.add_option("--port", dest="port", type="int",
                      help="Network port to send to", default=5554)
    (opts, args) = parser.parse_args()
    
    # NOTE: This is a hack to work with SITL
    my_ip = None
    bcast_ip = None
    if opts.device == 'lo':
        my_ip = '127.0.1.1'
        bcast_ip = '127.0.0.1'
    
    try:
        sock = Socket(0xff, opts.port, opts.device, my_ip, bcast_ip)
    except Exception:
        print "Sorry, couldn't start up the sending socket"
        sys.exit(1)
    
    counter = 0

    while True:
        message = acs_messages.Heartbeat()
        message.msg_dst = Socket.ID_BCAST_ALL
        message.msg_secs = 0
        message.msg_nsecs = 0
        message.counter = counter
        sock.send(message)

        counter += 1
        time.sleep(1)

