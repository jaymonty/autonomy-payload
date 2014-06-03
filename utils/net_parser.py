#!/usr/bin/env python

from optparse import OptionParser
import os
import sys
import time

# Load in ACS libraries
import os, inspect
LIB_ACS_PATH = os.path.realpath(
                   os.path.abspath(
                       os.path.join(
                           os.path.split(inspect.getfile(inspect.currentframe()))[0],
                           "../lib/acs")))
sys.path.insert(0, LIB_ACS_PATH)
import acs_messages
import acs_socket

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("pose_parser.py [options]")
    parser.add_option("--ip", dest="ip", 
                      help="Network address to listen on", default='')
    parser.add_option("--port", dest="port", type="int",
                      help="Network port to listen on", default=5554)
    parser.add_option("--repeat", dest="repeat",
                      action="store_true", default=False)
    (opts, args) = parser.parse_args()
    
    if not acs_socket.init('lo', opts.port, 0xff, None, opts.ip, '127.0.0.1'):
        print "Sorry, couldn't start up the listening socket"
        sys.exit(1)
    
    while True:
        message = acs_socket.recv()
        if message == None:  # No packets to receive, wait a bit
            time.sleep(0.1)
            continue
        elif message == False:  # Packet received, but ignored
            continue
        
        print "ID: %u > %u TYPE: %u TIME: %u.%09u" % \
            (message.msg_src, message.msg_dst, message.msg_type,
             message.msg_secs, message.msg_nsecs)
        
        if isinstance(message, acs_messages.Pose):
            print "\tLa: %f Lo: %f Al: %f Qx: %f Qy: %f Qz: %f Qw: %f" % \
                (message.lat, message.lon, message.alt, 
                 message.q_x, message.q_y, message.q_z, message.q_w)
        else:
            print "\t<Cannot decode this type>"
        
        # Debug code: repeat message (will take on false source ID)
        if opts.repeat:
            acs_socket.send(message)

