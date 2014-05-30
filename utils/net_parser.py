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
# Byte unpacking

'''
Packet header (in network byte order):
 - (8b)  Message type
 - (8b)  RESERVED
 - (16b) Aircraft ID
 - (32b) Seconds since epoch
 - (32b) Nanoseconds since last second

Pose message payload fields (type = 0x01):
 - (32b) Latitude (deg) * 1e07 (truncated)
 - (32b) Longitude (deg) * 1e07 (truncated)
 - (32b) Altitude (m) * 1e03 (truncated)
 - (32b) Quat X (?) * 1e09 (truncated)
 - (32b) Quat Y (?) * 1e09 (truncated)
 - (32b) Quat Z (?) * 1e09 (truncated)
 - (32b) Quat W (?) * 1e09 (truncated)
'''
HEADER_FORMAT = ">BBHLL"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
def unpack_header(data):
    return struct.unpack_from(HEADER_FORMAT, data, 0)

POSE_FORMAT = ">lllllll"
def unpack_pose(data):
    lat, lon, alt, q_x, q_y, q_z, q_w = \
        struct.unpack_from(POSE_FORMAT, data, HEADER_SIZE)
    lat = float(lat) / 1e07
    lon = float(lon) / 1e07
    alt = float(alt) / 1e03
    q_x = float(q_x) / 1e09
    q_y = float(q_y) / 1e09
    q_z = float(q_z) / 1e09
    q_w = float(q_w) / 1e09
    return lat, lon, alt, q_x, q_y, q_z, q_w

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
        
        m_type, _unused, m_id, m_secs, m_nsecs = unpack_header(dgram)
        
        print "TYPE: %u ID: %u TIME: %u.%09u" % \
            (m_type, m_id, m_secs, m_nsecs)
        
        if m_type == 0x01:
            lat, lon, alt, q_x, q_y, q_z, q_w = unpack_pose(dgram)
            print "\tLa: %f Lo: %f Al: %f Qx: %f Qy: %f Qz: %f Qw: %f" % \
                (lat, lon, alt, q_x, q_y, q_z, q_w)
        else:
            print "\t<Cannot decode this type>"
        
        # Debug code, reflect back with different id
        #d = bytearray(dgram)
        #struct.pack_into(">H", d, 2, 0x0002)
        #udp_sock.sendto(d, ('127.0.0.1', 5554))


