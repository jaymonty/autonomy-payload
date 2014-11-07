#!/usr/bin/env python

#-----------------------------------------------------------------------
# ACS Messages Library
# Mike Clement, 2014
#
# Some general info should go here!!

import struct

#-----------------------------------------------------------------------
# Helper functions and constants

# Convert a boolean into an all-0 or all-1 mask
def _bool8(val):
    if val:
        return 0xff
    return 0x00
def _bool16(val):
    if val:
        return 0xffff
    return 0x0000

# Specify bitmask for subswarm IDs
SUBSWARM_MASK = 0xE0  # 224-254 for subswarms
SUBSWARM_BITS = 0xFF - SUBSWARM_MASK

#-----------------------------------------------------------------------
# Base Message class

'''
Packet header format (all fields in network byte order):
 - (8b)  Message type
 - (8b)  Subswarm ID (low 5 bits; high 3 reserved)
 - (8b)  Source ID
 - (8b)  Destination ID
 - (32b) Seconds since Unix epoch
 - (16b) Milliseconds since last second
 - (16b) UNUSED
'''

class Message(object):
    # Define header parameters
    hdr_fmt = '>BBBBLHxx'
    hdr_size = struct.calcsize(hdr_fmt)

    def __init__(self):
        # Initialize common elements
        self.msg_type = None	# See type cases above
        self.msg_src = None	# Source ID (1-223 currently)
        self.msg_sub = None	# Source subswarm ID (0-30 currently)
        self.msg_dst = None	# Destination ID (1-255 currently)
        self.msg_secs = None	# Epoch seconds
        self.msg_nsecs = None	# Epoch nanoseconds (truncated to ms)
        # Add source IP and port, just for received messages (not serialized)
        self.msg_src_ip = None
        self.msg_src_port = None

        # Initialize payload component
        self._init_message()

    # Serialize a Message subtype
    def serialize(self):
        # Pack header
        hdr_tupl = (self.msg_type,
                    self.msg_sub & SUBSWARM_BITS,
                    self.msg_src,
                    self.msg_dst,
                    self.msg_secs,
                    self.msg_nsecs / 1e6)
        data = struct.pack(Message.hdr_fmt, *hdr_tupl)

        # Pack any payload
        data = data + self._pack()

        return data

    # Parse data into a Message subtype
    @staticmethod
    def parse(data):
        # Make sure we at least have a full header
        if len(data) < Message.hdr_size:
            raise Exception("invalid packet length")

        # Parse header fields
        try:
            msg_type, msg_sub, msg_src, msg_dst, msg_secs, msg_msecs = \
                struct.unpack_from(Message.hdr_fmt, data, 0)
        except Exception as ex:
            raise Exception("bad header: %s" % ex.args[0])

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
        elif msg_type == 0x89:
            msg = SetSubswarm()
        elif msg_type == 0x8A:
            msg = SetController()
        elif msg_type == 0x8B:
            msg = FollowerSetup()
        elif msg_type == 0x8C:
            msg = WPSequencerSetup()
        elif msg_type == 0xFE:
            msg = PayloadHeartbeat()
        elif msg_type == 0xFF:
            msg = PayloadShutdown()
        else:
            raise Exception("unknown type: %02X" % msg_type)

        # Populate header fields
        msg.msg_type = msg_type
        msg.msg_src = msg_src
        msg.msg_sub = msg_sub & SUBSWARM_BITS
        msg.msg_dst = msg_dst
        msg.msg_secs = msg_secs
        msg.msg_nsecs = msg_msecs * 1e6

        # Parse payload fields
        try:
            msg._unpack(data[Message.hdr_size:])
        except Exception as ex:
            raise Exception("bad payload: %s" % ex.args[0])

        return msg

#-----------------------------------------------------------------------
# Message definitions

# Example message type follows; copy and modify to need
'''
class Example(Message):
    def _init_message(self):
        # Define message type
        self.msg_type = 0x34	# See type cases above

        # If message is fixed-length, can define something like self.msg_fmt.
        # Nothing outside this class uses this, so do what makes sense here.
        self.msg_fmt = '>HH8s'

        # Define message fields (setting to None helps raise Exceptions later)
        self.foo = None         # Decimal foo's (e.g., 123.456)
        self.bar = None		# Integer bar's (e.g., 789)
        self.baz = None		# Variable-length string
        
    def _pack(self):
        # Convert message elements into pack-able fields and form tuple
        tupl = (int(self.foo * 1e03),
                int(self.bar),
                self.baz)

        # Pack into a byte string
        return struct.pack(self.msg_fmt, *tupl)
        
    def _unpack(self, data):
        # Unpack message contents (can do this incrementally for
        #  variable-length messages)
        tupl = struct.unpack_from(self.msg_fmt, data)
        
        # Place unpacked but unconverted fields into message elements
        self.foo = tupl[0] / 1e03
        self.bar = tupl[1]
        self.baz = tupl[2]
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
        
    def _pack(self):
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

        # Pack into a byte string
        return struct.pack(self.msg_fmt, *tupl)
        
    def _unpack(self, data):
        # Unpack payload into fields
        fields = struct.unpack_from(self.msg_fmt, data, 0)

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
        
    def _pack(self):
        # Convert message elements into pack-able fields and form tuple
        tupl = (int(self.lat * 1e07),
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

        # Pack into a byte string
        return struct.pack(self.msg_fmt, *tupl)
        
    def _unpack(self, data):
        # Unpack payload into fields
        fields = struct.unpack_from(self.msg_fmt, data, 0)

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
        self.msg_type = 0x80
        self.msg_fmt = '>L'

        self.counter = None         # User-definable counter (UInt32)

    def _pack(self):
        tupl = (int(self.counter),)
        return struct.pack(self.msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(self.msg_fmt, data, 0)
        self.counter = int(fields[0]) 

class Arm(Message):
    def _init_message(self):
        self.msg_type = 0x81
        self.msg_fmt = '>B3x'

        self.enable = None         # Boolean
        # 3 padding bytes = 0x00
        
    def _pack(self):
        tupl = (int(self.enable),)
        return struct.pack(self.msg_fmt, *tupl)
        
    def _unpack(self, data):
        fields = struct.unpack_from(self.msg_fmt, data, 0)
        self.enable = bool(fields[0])

class Mode(Message):
    def _init_message(self):
        self.msg_type = 0x82
        self.msg_fmt = '>B3x'

        self.mode = None         # Mode ID (0-15)
        # 3 padding bytes = 0x00

    def _pack(self):
        tupl = (int(self.mode),)
        return struct.pack(self.msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(self.msg_fmt, data, 0)
        self.mode = int(fields[0]) 

class Land(Message):
    def _init_message(self):
        self.msg_type = 0x83
        self.msg_fmt = ''

    def _pack(self):
        return ''

    def _unpack(self, data):
        pass

class LandAbort(Message):
    def _init_message(self):
        self.msg_type = 0x84
        self.msg_fmt = '>h2x'

        self.alt = None         # Waive-off altitude (approx +/-32000)
        # 2 padding bytes = 0x0000

    def _pack(self):
        tupl = (int(self.alt),)
        return struct.pack(self.msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(self.msg_fmt, data, 0)
        self.alt = int(fields[0]) 

class GuidedGoto(Message):
    def _init_message(self):
        self.msg_type = 0x85
        self.msg_fmt = '>lll'

        self.lat = None		# Decimal degrees (e.g. 35.123456)
        self.lon = None		# Decimal degrees (e.g. -120.123456)
        self.alt = None		# Decimal meters MSL (WGS84)

    def _pack(self):
        tupl = (int(self.lat * 1e07),
                int(self.lon * 1e07),
                int(self.alt * 1e03))
        return struct.pack(self.msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(self.msg_fmt, data, 0)
        self.lat = fields[0] / 1e07
        self.lon = fields[1] / 1e07
        self.alt = fields[2] / 1e03

class WaypointGoto(Message):
    def _init_message(self):
        self.msg_type = 0x86
        self.msg_fmt = '>H2x'

        self.index = None         # Waypoint index (0-65535)
        # 2 padding bytes = 0x0000

    def _pack(self):
        tupl = (int(self.index),)
        return struct.pack(self.msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(self.msg_fmt, data, 0)
        self.index = int(fields[0])

class SlaveSetup(Message):
    def _init_message(self):
        self.msg_type = 0x87
        self.msg_fmt = '>B100p'

        self.enable = None         # Boolean
        self.channel = None        # Pascal String

    def _pack(self):
        tupl = (int(self.enable),
                str(self.channel))
        return struct.pack(self.msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(self.msg_fmt, data, 0)
        self.enable = bool(fields[0]) 
        self.channel = str(fields[1])

class FlightReady(Message):
    def _init_message(self):
        self.msg_type = 0x88
        self.msg_fmt = '>B3x'

        self.raedy = None         # Boolean
        # 3 padding bytes = 0x00

    def _pack(self):
        tupl = (int(self.ready),)
        return struct.pack(self.msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(self.msg_fmt, data, 0)
        self.ready = bool(fields[0]) 

class SetSubswarm(Message):
    def _init_message(self):
        self.msg_type = 0x89
        self.msg_fmt = '>B3x'

        self.subswarm = None         # New subswarm ID
        # 3 padding bytes = 0x00

    def _pack(self):
        tupl = (int(self.subswarm),)
        return struct.pack(self.msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(self.msg_fmt, data, 0)
        self.subswarm = int(fields[0]) 

class SetController(Message):
    def _init_message(self):
        self.msg_type = 0x8A
        self.msg_fmt = '>B3x'

        self.controller = None     # Numeric ID of controller type
        # 3 padding bytes

    def _pack(self):
        tupl = (int(self.controller),)
        return struct.pack(self.msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(self.msg_fmt, data, 0)
        self.controller = int(fields[0]) 

class FollowerSetup(Message):
    def _init_message(self):
        self.msg_type = 0x8B
        self.msg_fmt = '>hhhBB'

        self.follow_range = None    # Distance behind leader (meters)
        self.offset_angle = None    # Offset angle from leader (radians, 0=astern)
        self.control_alt = None     # Relative or absolute altitude (meters)
        self.leader_id = None       # ID of aircraft to follow
        self.alt_mode = None        # 0=absolute, 1=relative

    def _pack(self):
        tupl = (int(self.follow_range),
                int(self.offset_angle * 1e2),
                int(self.control_alt),
                self.leader_id,
                self.alt_mode)
        return struct.pack(self.msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(self.msg_fmt, data, 0)
        self.follow_range = float(fields[0])
        self.offset_angle = float(fields[1]) / 1e2
        self.control_alt = float(fields[3])
        self.leader_id = fields[4]
        self.alt_mode = fields[5]

class WPSequencerSetup(Message):
    def _init_message(self):
        self.msg_type = 0x8C
        self.msg_fmt_base = '>BxH'
        self.msg_fmt_wp = '>lll'

        # 1 byte                 # Count of LLA tuples in wp_list (0-255)
        # 1 padding byte
        self.seq = None          # Task sequence number (optional but should increment)
        self.wp_list = []        # List of LLA tuples (degrees, degrees, meters)

    def _pack(self):
        tupl = (len(self.wp_list), self.seq)
        for lla in self.wp_list:
            tupl += (int(lla[0] * 1e7),
                     int(lla[1] * 1e7),
                     int(lla[2] * 1e3))
        fmt = self.msg_fmt_base + len(self.wp_list) * self.msg_fmt_wp.lstrip('>')
        return struct.pack(fmt, tupl)

    def _unpack(self, data):
        (wp_count, self.seq) = struct.unpack_from(self.msg_fmt_base, data, 0)
        offset = struct.calcsize(self.msg_fmt_base)
        for wp in wp_count:
            lla = struct.unpack_from(self.msg_fmt_wp, data, offset)
            wp_list.append(lla)
            offset += struct.calcsize(self.msg_fmt_wp)

class PayloadHeartbeat(Message):
    def _init_message(self):
        self.msg_type = 0xFE
        self.msg_fmt = '>B3x'

        self.enable = None         # Boolean
        # 3 padding bytes = 0x00

    def _pack(self):
        tupl = (int(self.enable),)
        return struct.pack(self.msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(self.msg_fmt, data, 0)
        self.enable = bool(fields[0])

class PayloadShutdown(Message):
    def _init_message(self):
        self.msg_type = 0xFF

    def _pack(self):
        return ''

    def _unpack(self, data):
        pass

