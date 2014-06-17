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
from acs_socket import Socket

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("pose_parser.py [options]")
    parser.add_option("--device", dest="device", 
                      help="Network device to listen on", default='')
    parser.add_option("--port", dest="port", type="int",
                      help="Network port to listen on", default=5554)
    parser.add_option("--repeat", dest="repeat",
                      action="store_true", default=False)
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
        print "Sorry, couldn't start up the listening socket"
        sys.exit(1)
    
    while True:
        message = sock.recv()
        if message == None:  # No packets to receive, wait a bit
            time.sleep(0.1)
            continue
        elif message == False:  # Packet received, but ignored
            print "<Ignored packet>"
            continue
        
        print "ID: %u > %u TYPE: %02X TIME: %u.%09u" % \
            (message.msg_src, message.msg_dst, message.msg_type,
             message.msg_secs, message.msg_nsecs)
        
        if isinstance(message, acs_messages.Pose):
            print "\tLa: %f Lo: %f Al: %f Qx: %f Qy: %f Qz: %f Qw: %f" % \
                (message.lat, message.lon, message.alt, 
                 message.q_x, message.q_y, message.q_z, message.q_w)
        elif isinstance(message, acs_messages.FlightStatus):
            print "\tMo: %u Wp: %u Fl: %u%u%u%u%u%u%u Sa: %u Br: %d Bv: %d Bc: %d As: %f Ar: %d Gh: %u" % \
                (message.mode, int(message.mis_cur), int(message.armed), int(message.ok_ahrs),
                 int(message.ok_as), int(message.ok_gps), int(message.ok_ins),
                 int(message.ok_mag), int(message.ok_pwr),
                 message.gps_sats, message.batt_rem, message.batt_vcc,
                 message.batt_cur, message.airspeed, message.alt_rel,
                 message.gps_hdop)
        else:
            print "\t<I don't know how to decode this type>"
        
        # Debug code: repeat message (will take on false source ID)
        if opts.repeat:
            sock.send(message)

