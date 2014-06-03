#!/usr/bin/env python

#-----------------------------------------------------------------------
# ACS Messages Library
# Mike Clement, 2014
#
# Some general info should go here!!

import struct

#-----------------------------------------------------------------------
# Base Message class

'''
Packet header format (all fields in network byte order):
 - (8b)  Message type
 - (8b)  RESERVED
 - (8b)  Source ID
 - (8b)  Destination ID
 - (32b) Seconds since Unix epoch
 - (32b) Nanoseconds since last second
'''

# Cache message sizes so we only calculate once each
_msg_sizes = {}
def _get_msg_size(fmt):
    if fmt not in _msg_sizes:
        _msg_sizes[fmt] = struct.calcsize(fmt)
    return _msg_sizes[fmt]

class Message():
    # Define header parameters
    hdr_fmt = '>BBBBLL'
    hdr_size = struct.calcsize(hdr_fmt)
    
    def __init__(self):
        # Set instance variables from class variables
        self.hdr_fmt = Message.hdr_fmt
        self.hdr_size = Message.hdr_size
        # Initialize common elements
        self.msg_type = None
        self.msg_src = None
        self.msg_dst = None
        self.msg_secs = None
        self.msg_nsecs = None
        
        # Initialize payload component
        self._init_message()
        self.msg_size = _get_msg_size(self.msg_fmt)
        
    def build_hdr_tuple(self):
        return (self.msg_type,
                0x00,
                self.msg_src,
                self.msg_dst,
                self.msg_secs,
                self.msg_nsecs)
    
    def parse_hdr_tuple(self, fields):
        self.msg_type  = fields[0]
        # fields[1] is unused
        self.msg_src   = fields[2]
        self.msg_dst   = fields[3]
        self.msg_secs  = fields[4]
        self.msg_nsecs = fields[5]

#-----------------------------------------------------------------------
# Message definitions

# Must add each new message type here, used for receiving/parsing
def generate_message_object(msg_type):
    if msg_type == 0x00:
        return None
    elif msg_type == 0x01:
        return Pose()
    else:
       return None

class Pose(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x01
        self.msg_fmt = '>lllllll'
        # Define message fields (setting to None helps raise Exceptions later)
        self.lat = None		# Decimal degrees (e.g. 35.123456)
        self.lon = None		# Decimal degrees (e.g. -120.123456)
        self.alt = None		# Decimal meters MSL (WGS84)
        self.q_x = None		# Quaternion X
        self.q_y = None		# Quaternion Y
        self.q_z = None		# Quaternion Z
        self.q_w = None		# Quaternion W
        
    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return (int(self.lat * 1e07),
                int(self.lon * 1e07),
                int(self.alt * 1e03),
                int(self.q_x * 1e09),
                int(self.q_y * 1e09),
                int(self.q_z * 1e09),
                int(self.q_w * 1e09))
        
    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.lat = fields[0] / 1e07
        self.lon = fields[1] / 1e07
        self.alt = fields[2] / 1e03
        self.q_x = fields[3] / 1e09
        self.q_y = fields[4] / 1e09
        self.q_z = fields[5] / 1e09
        self.q_w = fields[6] / 1e09



