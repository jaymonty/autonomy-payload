#!/usr/bin/env python

#-----------------------------------------------------------------------
# ACS Messages Library
# Mike Clement, 2014
#
# Some general info should go here!!

import struct

#-----------------------------------------------------------------------
# Base class for all messages

'''
Packet header format (all fields in network byte order):
 - (8b)  Message type
 - (8b)  RESERVED
 - (8b)  Source ID
 - (8b)  Destination ID
 - (32b) Seconds since Unix epoch
 - (32b) Nanoseconds since last second
'''

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
        
    def build_hdr_tuple(self):
        return (self.msg_type,
                0x00,
                self.msg_src,
                self.msg_dst,
                self.msg_secs,
                self.msg_nsecs)

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
    # Define message type parameters
    msg_type = 0x01
    msg_fmt = '>lllllll'
    msg_size = struct.calcsize(msg_fmt)
    
    def _init_message(self):
        # Set instance variables from class variables
        self.msg_type = Pose.msg_type
        self.msg_fmt = Pose.msg_fmt
        self.msg_size = Pose.msg_size
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



