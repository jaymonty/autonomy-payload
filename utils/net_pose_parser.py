#!/usr/bin/env python

import netifaces
from optparse import OptionParser
import socket
import struct
import sys
import time

#-----------------------------------------------------------------------
# UDP socket and info
udp_sock = None

def sock_init(ip, port):
    global udp_sock
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind((ip, port))
    return True

def sock_recv(max_size=1024):
    data = None
    try:
        data, (ip, port) = udp_sock.recvfrom(max_size, socket.MSG_DONTWAIT)
    except Exception:
        return None
    return data

#-----------------------------------------------------------------------
# Byte packing and unpacking

PACKET_FORMAT = ">HHLLLlllllll"
'''
Packet fields (in network byte order):
 - (16b) Aircraft ID (1..65535)
 - (16b) RESERVED
 - (32b) Message sequence number (1..4Bil)
 - (32b) Seconds since epoch
 - (32b) Nanoseconds since last second
 - (32b) Latitude (deg) * 1e07 (truncated)
 - (32b) Longitude (deg) * 1e07 (truncated)
 - (32b) Altitude (m) * 1e03 (truncated)
 - (32b) Quat X (?) * 1e09 (truncated)
 - (32b) Quat Y (?) * 1e09 (truncated)
 - (32b) Quat Z (?) * 1e09 (truncated)
 - (32b) Quat W (?) * 1e09 (truncated)
'''

def pose_unpack(data):
    acid, unused1, seq, secs, nsecs, lat, lon, alt, q_x, q_y, q_z, q_w \
        = struct.unpack(PACKET_FORMAT, data)
    lat = float(lat) / 1e07
    lon = float(lon) / 1e07
    alt = float(alt) / 1e03
    q_x = float(q_x) / 1e09
    q_y = float(q_y) / 1e09
    q_z = float(q_z) / 1e09
    q_w = float(q_w) / 1e09
    return acid, seq, secs, nsecs, lat, lon, alt, q_x, q_y, q_z, q_w

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("pose_parser.py [options]")
    parser.add_option("--ip", dest="ip", 
                      help="Network address to listen on", default='')
    parser.add_option("--port", dest="port", type="int",
                      help="Network port to listen on", default=5554)
    (opts, args) = parser.parse_args()
    
    if not sock_init(opts.ip, opts.port):
        print "Sorry, couldn't start up the listening socket"
        sys.exit(1)
    
    while True:
        dgram = sock_recv()
        if dgram == None:  # Nothing received; wait a bit
            time.sleep(0.1)
            continue
        
        acid, seq, secs, nsecs, lat, lon, alt, q_x, q_y, q_z, q_w \
            = pose_unpack(dgram)
        
        print "ID: %u SEQ: %u TIME: %u.%09u" % \
            (acid, seq, secs, nsecs)
        print "\tLa: %f Lo: %f Al: %f Qx: %f Qy: %f Qz: %f Qw: %f" % \
            (lat, lon, alt, q_x, q_y, q_z, q_w)

