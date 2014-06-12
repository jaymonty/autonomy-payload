#!/usr/bin/env python

#-----------------------------------------------------------------------
# ACS Messages Library
# Mike Clement, 2014
#
# Some general info should go here!!

import struct

#-----------------------------------------------------------------------
# Base Message class and helper functions

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

# Convert a boolean into an all-0 or all-1 mask
def _bool8(val):
    if val:
        return 0xff
    return 0x00
def _bool16(val):
    if val:
        return 0xffff
    return 0x0000

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
        return FlightStatus()
    elif msg_type == 0x01:
        return Pose()
    else:
       return None

# Example message type follows; copy and modify to need
'''
class Example(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x34	# Must be unique
        self.msg_fmt = '>HH'	# See Python struct package
        # Define message fields (setting to None helps raise Exceptions later)
        self.foo = None         # Decimal foo's (e.g., 123.456)
        self.bar = None		# Integer bar's (e.g., 789)
        
    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return (int(self.foo * 1e03),
                int(self.bar))
        
    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.foo = fields[0] / 1e03
        self.bar = fields[1]
'''

class FlightStatus(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x00
        self.msg_fmt = '>HBBHHhhHH'
        # Define message fields (setting to None helps raise Exceptions later)
        self.mode = None	# Aircraft guidance mode (0-15, see enum)
        self.armed = None	# Boolean: Throttle Armed?
        self.ok_ahrs = None	# Boolean: AHRS OK?
        self.ok_as = None	# Boolean: Airspeed Sensor OK?
        self.ok_gps = None	# Boolean: GPS sensor OK?
        self.ok_ins = None	# Boolean: INS sensor OK?
        self.ok_mag = None	# Boolean: Magnetometer OK?
        self.ok_pwr = None	# Boolean: Power OK?
        self.gps_sats = None	# Number of satellites (int, 0-255)
        self.batt_rem = None	# Battery % remaining (int, 0-100)
        self.batt_vcc = None	# Battery Voltage (int, mV)
        self.batt_cur = None	# Battery Current (int, mA)
        self.airspeed = None	# Airspeed (float, m/s)
        self.alt_rel = None	# AGL (int, millimeters)
        self.gps_hdop = None	# GPS HDOP (int, 0-65535)
        self.mis_cur = None	# Current mission (waypoint) index (0-65535)
        
    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        mode_and_flags = self.mode << 12 \
                       | (0x0800 & _bool16(self.armed)) \
                       | (0x0400 & _bool16(self.ok_ahrs)) \
                       | (0x0200 & _bool16(self.ok_as)) \
                       | (0x0100 & _bool16(self.ok_gps)) \
                       | (0x0080 & _bool16(self.ok_ins)) \
                       | (0x0040 & _bool16(self.ok_mag)) \
                       | (0x0020 & _bool16(self.ok_pwr)) \
                       & 0xffe0  # Zeroize unused bits
        batt_rem = 255
        if 0 <= self.batt_rem <= 100:  # Set invalid values to max unsigned
            batt_rem = self.batt_rem
        batt_vcc = 65535
        if self.batt_vcc >= 0:
            batt_vcc = self.batt_vcc * 1e02
        batt_cur = 65535
        if self.batt_cur >= 0:
            batt_cur = self.batt_cur * 1e02
        tupl = (mode_and_flags,
                int(self.gps_sats),
                int(batt_rem),
                int(batt_vcc),
                int(batt_cur),
                int(self.airspeed * 1e02),  # TODO: Are these large enough?
                int(self.alt_rel / 1e02),
                int(self.gps_hdop),
                int(self.mis_cur))
        #print tupl
        return tupl
        
    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.mode = fields[0] >> 12
        self.armed = bool(fields[0] & 0x0800)
        self.ok_ahrs = bool(fields[0] & 0x0400)
        self.ok_as = bool(fields[0] & 0x0200)
        self.ok_gps = bool(fields[0] & 0x0100)
        self.ok_ins = bool(fields[0] & 0x0080)
        self.ok_mag = bool(fields[0] & 0x0040)
        self.ok_pwr = bool(fields[0] & 0x0020)
        self.gps_sats = fields[1]
        self.batt_rem = fields[2]
        if self.batt_rem == 255:  # Account for invalid values
            self.batt_rem = -1
        self.batt_vcc = -1
        if fields[3] != 65535:  # Account for invalid values
            self.batt_vcc = fields[3] / 1e02
        self.batt_cur = -1
        if fields[4] != 65535:  # Account for invalid values
            self.batt_cur = fields[4] / 1e02
        self.airspeed = fields[5] / 1e02
        self.alt_rel = fields[6] * 1e02
        self.gps_hdop = fields[7]
        self.mis_cur = fields[8]

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

class Heartbeat(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x80
        self.msg_fmt = '>BBH'
        # Define message fields (setting to None helps raise Exceptions later)
        self.enable = None         # Boolean
        # 3 padding bytes = 0x00

    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return (int(self.enable),
                0x00,
                0x0000)

    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.enable = bool(fields[0]) 

class Arm(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x81
        self.msg_fmt = '>BBH'
        # Define message fields (setting to None helps raise Exceptions later)
        self.enable = None         # Boolean
        # 3 padding bytes = 0x00
        
    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return (int(self.enable),
                0x00,
                0x0000)
        
    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.enable = bool(fields[0])

class Mode(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x82
        self.msg_fmt = '>BBH'
        # Define message fields (setting to None helps raise Exceptions later)
        self.mode = None         # Mode ID (0-15)
        # 3 padding bytes = 0x00

    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return (int(self.mode),
                0x00,
                0x0000)

    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.mode = int(fields[0]) 

class Land(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x83
        self.msg_fmt = ''
        # Define message fields (setting to None helps raise Exceptions later)

    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return None

    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        True

class LandAbort(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x84
        self.msg_fmt = '>hH'
        # Define message fields (setting to None helps raise Exceptions later)
        self.alt = None         # Waive-off altitude (approx +/-32000)
        # 2 padding bytes = 0x0000

    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return (int(self.alt),
                0x0000)

    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.alt = int(fields[0]) 

class GuidedGoto(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x85
        self.msg_fmt = '>lll'
        # Define message fields (setting to None helps raise Exceptions later)
        self.lat = None		# Decimal degrees (e.g. 35.123456)
        self.lon = None		# Decimal degrees (e.g. -120.123456)
        self.alt = None		# Decimal meters MSL (WGS84)

    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return (int(self.lat * 1e07),
                int(self.lon * 1e07),
                int(self.alt * 1e03))

    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.lat = fields[0] / 1e07
        self.lon = fields[1] / 1e07
        self.alt = fields[2] / 1e03

class WaypointGoto(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x86
        self.msg_fmt = '>HH'
        # Define message fields (setting to None helps raise Exceptions later)
        self.index = None         # Waypoint index (0-65535)
        # 2 padding bytes = 0x0000

    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return (int(self.index),
                0x0000)

    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.index = int(fields[0])

class PayloadShutdown(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0xFF
        self.msg_fmt = ''
        # Define message fields (setting to None helps raise Exceptions later)

    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return None

    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        True

