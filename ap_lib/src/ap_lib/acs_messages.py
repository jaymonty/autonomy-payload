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

# Parse data into a Message subtype
def parse(data):
    # Make sure we at least have a full header
    if len(data) < Message.hdr_size:
        print "header too small"
        return None

    # Parse header fields
    try:
        msg_type, _unused, msg_src, msg_dst, msg_secs, msg_nsecs = \
            struct.unpack_from(Message.hdr_fmt, data, 0)
    except:
        print "header bad"
        return None

    # Create corresponding subtype
    if msg_type == 0x00:
        msg = FlightStatus()
    elif msg_type == 0x01:
        msg = Pose()
    elif msg_type == 0x80:
        msg = Heartbeat()
    elif msg_type == 0x81:
        msg = Arm()
    elif msg_type == 0x82:
        msg = Mode()
    elif msg_type == 0x83:
        msg = Land()
    elif msg_type == 0x84:
        msg = LandAbort()
    elif msg_type == 0x85:
        msg = GuidedGoto()
    elif msg_type == 0x86:
        msg = WaypointGoto()
    elif msg_type == 0x87:
        msg = SlaveSetup()
    elif msg_type == 0x88:
        msg = FlightReady()
    elif msg_type == 0xFE:
        msg = PayloadHeartbeat()
    elif msg_type == 0xFF:
        msg = PayloadShutdown()
    else:
        print "unknown type: %02X" % msg_type
        return None

    # Make sure we have right number of bytes
    if len(data) != msg.hdr_size + msg.msg_size:
        print "payload wrong size"
        return None

    # Populate header fields
    msg.msg_type = msg_type
    msg.msg_src = msg_src
    msg.msg_dst = msg_dst
    msg.msg_secs = msg_secs
    msg.msg_nsecs = msg_nsecs

    # If subtype has no payload fields, we're done
    if not msg.msg_size:
        return msg

    # Parse payload fields
    try:
        fields = struct.unpack_from(msg.msg_fmt, data, msg.hdr_size)
        msg.parse_tuple(fields)
        return msg
    except:
        print "payload bad"
        return None

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
        # Add source IP and port, just for received messages (not serialized)
        self.msg_src_ip = None
        self.msg_src_port = None

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

    def serialize(self):
        data = struct.pack(self.hdr_fmt, *self.build_hdr_tuple())
        if self.msg_size:
            data = data + struct.pack(self.msg_fmt, *self.build_tuple())
        return data

#-----------------------------------------------------------------------
# Message definitions

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
        self.msg_fmt = '>HBBHHhhHH16s'
        # Define message fields (setting to None helps raise Exceptions later)
        self.mode = None	# Aircraft guidance mode (0-15, see enum)
        self.armed = None	# Boolean: Throttle Armed?
        self.ready = None	# Boolean: Ready for flight? (user-settable)
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
        self.name = None	# Friendly name of aircraft (16 chars max)
        
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
                       | (0x0010 & _bool16(self.ready)) \
                       & 0xfff0  # Zeroize unused bits
        batt_rem = 255
        if 0 <= self.batt_rem <= 100:  # Set invalid values to max unsigned
            batt_rem = self.batt_rem
        batt_vcc = 65535
        if self.batt_vcc >= 0:
            batt_vcc = self.batt_vcc
        batt_cur = 65535
        if self.batt_cur >= 0:
            batt_cur = self.batt_cur
        tupl = (mode_and_flags,
                int(self.gps_sats),
                int(batt_rem),
                int(batt_vcc),
                int(batt_cur),
                int(self.airspeed * 1e02),  # TODO: Are these large enough?
                int(self.alt_rel / 1e02),
                int(self.gps_hdop),
                int(self.mis_cur),
                self.name)
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
        self.ready = bool(fields[0] & 0x0010)
        self.gps_sats = fields[1]
        self.batt_rem = fields[2]
        if self.batt_rem == 255:  # Account for invalid values
            self.batt_rem = -1
        self.batt_vcc = -1
        if fields[3] != 65535:  # Account for invalid values
            self.batt_vcc = fields[3]
        self.batt_cur = -1
        if fields[4] != 65535:  # Account for invalid values
            self.batt_cur = fields[4]
        self.airspeed = fields[5] / 1e02
        self.alt_rel = fields[6] * 1e02
        self.gps_hdop = fields[7]
        self.mis_cur = fields[8]
        self.name = str(fields[9]).strip(chr(0x0))

class Pose(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x01
        self.msg_fmt = '>lllllllhhhhhh'
        # Define message fields (setting to None helps raise Exceptions later)
        self.lat = None		# Decimal degrees (e.g. 35.123456)
        self.lon = None		# Decimal degrees (e.g. -120.123456)
        self.alt = None		# Decimal meters MSL (WGS84)
        self.q_x = None		# Quaternion X
        self.q_y = None		# Quaternion Y
        self.q_z = None		# Quaternion Z
        self.q_w = None		# Quaternion W
        self.vlx = None		# Linear velocity x (cm/s)
        self.vly = None		# Linear velocity y (cm/s)
        self.vlz = None		# Linear velocity z (cm/s)
        self.vax = None		# Angular velocity x (rad/s * 100)
        self.vay = None		# Angular velocity y (rad/s * 100)
        self.vaz = None		# Angular velocity z (rad/s * 100)
        
    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return (int(self.lat * 1e07),
                int(self.lon * 1e07),
                int(self.alt * 1e03),
                int(self.q_x * 1e09),
                int(self.q_y * 1e09),
                int(self.q_z * 1e09),
                int(self.q_w * 1e09),
                int(self.vlx * 1e02),
                int(self.vly * 1e02),
                int(self.vlz * 1e02),
                int(self.vax * 1e02),
                int(self.vay * 1e02),
                int(self.vaz * 1e02))
        
    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.lat = fields[0] / 1e07
        self.lon = fields[1] / 1e07
        self.alt = fields[2] / 1e03
        self.q_x = fields[3] / 1e09
        self.q_y = fields[4] / 1e09
        self.q_z = fields[5] / 1e09
        self.q_w = fields[6] / 1e09
        self.vlx = fields[7] / 1e02
        self.vly = fields[8] / 1e02
        self.vlz = fields[9] / 1e02
        self.vax = fields[10] / 1e02
        self.vay = fields[11] / 1e02
        self.vaz = fields[12] / 1e02

class Heartbeat(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x80
        self.msg_fmt = '>L'
        # Define message fields (setting to None helps raise Exceptions later)
        self.counter = None         # User-definable counter (UInt32)
        # 3 padding bytes = 0x00

    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return (int(self.counter),)

    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.counter = int(fields[0]) 

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

class SlaveSetup(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x87
        self.msg_fmt = '>B100p'
        # Define message fields (setting to None helps raise Exceptions later)
        self.enable = None         # Boolean
        self.channel = None        # Pascal String

    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return (int(self.enable),
                str(self.channel))

    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.enable = bool(fields[0]) 
        self.channel = str(fields[1])

class FlightReady(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0x88
        self.msg_fmt = '>BBH'
        # Define message fields (setting to None helps raise Exceptions later)
        self.raedy = None         # Boolean
        # 3 padding bytes = 0x00

    def build_tuple(self):
        # Convert message elements into pack-able fields and form tuple
        return (int(self.ready),
                0x00,
                0x0000)

    def parse_tuple(self, fields):
        # Place unpacked but unconverted fields into message elements
        self.ready = bool(fields[0]) 

class PayloadHeartbeat(Message):
    def _init_message(self):
        # Define message type parameters
        self.msg_type = 0xFE
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

