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

# Bitmasks
SUBSWARM_MASK = 0x1F  # low 5 bits (of 8)
FL_REL_MASK = 0x80    # high bit (of 8)
FL_SYN_MASK = 0x40    # second bit (of 8)

#-----------------------------------------------------------------------
# Base Message class

'''
Packet header format (all fields in network byte order):
 - (8b)  Message type
 - (8b)  Flags (high-3) + Subswarm ID (low-5)
          - 0x80 - Message sent reliably (send-buffered)
          - 0x40 - SYNchronize with remote side
          - 0x20 - UNUSED
 - (8b)  Source ID
 - (8b)  Destination ID
 - (16b) Reliable messaging sequence number
 - (16b) Reliable messaging acknowledgment number (+1)
 - (32b) Seconds since Unix epoch
 - (16b) Milliseconds since last second
 - (16b) UNUSED
'''

class Message(object):
    # Define header parameters
    hdr_fmt = '>BBBBHHLH2x'
    hdr_size = struct.calcsize(hdr_fmt)

    def __init__(self):
        # Initialize instance msg_type from class msg_type
        # (done for compatibility only)
        self.msg_type = type(self).msg_type

        # Initialize other header fields
        self.msg_fl_rel = False # Reliable flag
        self.msg_fl_syn = False # SYN flag
        self.msg_sub = None	# Source subswarm ID (0-30 currently)
        self.msg_src = None	# Source ID (1-223 currently)
        self.msg_dst = None	# Destination ID (1-255 currently)
        self.msg_seq = None     # Highest reliable seqnum sent to dest
        self.msg_ack = None     # Highest reliable seqnum seen from dest + 1
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
        hdr_tupl = (type(self).msg_type,
                    (self.msg_sub & SUBSWARM_MASK) | \
                    (_bool8(self.msg_fl_rel) & FL_REL_MASK) | \
                    (_bool8(self.msg_fl_syn) & FL_SYN_MASK),
                    self.msg_src,
                    self.msg_dst,
                    self.msg_seq,
                    self.msg_ack,
                    self.msg_secs,
                    int(self.msg_nsecs / 1e6))

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
            msg_type, msg_sub, msg_src, msg_dst, msg_seq, msg_ack, msg_secs, msg_msecs = \
                struct.unpack_from(Message.hdr_fmt, data, 0)
        except Exception as ex:
            raise Exception("bad header: %s" % ex.args[0])

        # Create corresponding subtype
        typelist = {sc.msg_type : sc for sc in Message.__subclasses__()}
        if msg_type not in typelist:
            raise Exception("unknown type: %02X" % msg_type)
        msg = typelist[msg_type]()

        # Populate header fields
        msg.msg_src = msg_src
        msg.msg_fl_rel = bool(msg_sub & FL_REL_MASK)
        msg.msg_fl_syn = bool(msg_sub & FL_SYN_MASK)
        msg.msg_sub = msg_sub & SUBSWARM_MASK
        msg.msg_dst = msg_dst
        msg.msg_seq = msg_seq
        msg.msg_ack = msg_ack
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
    # Define message type
    msg_type = 0x34	# See type cases above

    # If message is fixed-length, can define something like self.msg_fmt.
    # Nothing outside this class uses this, so do what makes sense here.
    msg_fmt = '>HH8s'

    def _init_message(self):
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
        return struct.pack(type(self).msg_fmt, *tupl)
        
    def _unpack(self, data):
        # Unpack message contents (can do this incrementally for
        #  variable-length messages)
        tupl = struct.unpack_from(type(self).msg_fmt, data)
        
        # Place unpacked but unconverted fields into message elements
        self.foo = tupl[0] / 1e03
        self.bar = tupl[1]
        self.baz = tupl[2]
'''

class FlightStatus(Message):
    # Define message type parameters
    msg_type = 0x00
    msg_fmt = '>HBBHHhhHH16s'

    def _init_message(self):
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
        return struct.pack(type(self).msg_fmt, *tupl)
        
    def _unpack(self, data):
        # Unpack payload into fields
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)

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
    # Define message type parameters
    msg_type = 0x01
    msg_fmt = '>lllllllhhhhhh'

    def _init_message(self):
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
        return struct.pack(type(self).msg_fmt, *tupl)
        
    def _unpack(self, data):
        # Unpack payload into fields
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)

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
    msg_type = 0x80
    msg_fmt = '>L'

    def _init_message(self):
        self.counter = None         # User-definable counter (UInt32)

    def _pack(self):
        tupl = (int(self.counter),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.counter = int(fields[0]) 

class Arm(Message):
    msg_type = 0x81
    msg_fmt = '>B3x'

    def _init_message(self):
        self.enable = None         # Boolean
        # 3 padding bytes = 0x00
        
    def _pack(self):
        tupl = (int(self.enable),)
        return struct.pack(type(self).msg_fmt, *tupl)
        
    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.enable = bool(fields[0])

class Mode(Message):
    msg_type = 0x82
    msg_fmt = '>B3x'

    def _init_message(self):
        self.mode = None         # Mode ID (0-15)
        # 3 padding bytes = 0x00

    def _pack(self):
        tupl = (int(self.mode),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.mode = int(fields[0]) 

class Land(Message):
    msg_type = 0x83

    def _init_message(self):
        pass

    def _pack(self):
        return ''

    def _unpack(self, data):
        pass

class LandAbort(Message):
    msg_type = 0x84
    msg_fmt = '>h2x'

    def _init_message(self):
        self.alt = None         # Waive-off altitude (approx +/-32000)
        # 2 padding bytes = 0x0000

    def _pack(self):
        tupl = (int(self.alt),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.alt = int(fields[0]) 

class GuidedGoto(Message):
    msg_type = 0x85
    msg_fmt = '>lll'

    def _init_message(self):
        self.lat = None		# Decimal degrees (e.g. 35.123456)
        self.lon = None		# Decimal degrees (e.g. -120.123456)
        self.alt = None		# Decimal meters MSL (WGS84)

    def _pack(self):
        tupl = (int(self.lat * 1e07),
                int(self.lon * 1e07),
                int(self.alt * 1e03))
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.lat = fields[0] / 1e07
        self.lon = fields[1] / 1e07
        self.alt = fields[2] / 1e03

class WaypointGoto(Message):
    msg_type = 0x86
    msg_fmt = '>H2x'

    def _init_message(self):
        self.index = None         # Waypoint index (0-65535)
        # 2 padding bytes = 0x0000

    def _pack(self):
        tupl = (int(self.index),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.index = int(fields[0])

class SlaveSetup(Message):
    msg_type = 0x87
    msg_fmt = '>B100p'

    def _init_message(self):
        self.enable = None         # Boolean
        self.channel = None        # Pascal String

    def _pack(self):
        tupl = (int(self.enable),
                str(self.channel))
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.enable = bool(fields[0]) 
        self.channel = str(fields[1])

class FlightReady(Message):
    msg_type = 0x88
    msg_fmt = '>B3x'

    def _init_message(self):
        self.raedy = None         # Boolean
        # 3 padding bytes = 0x00

    def _pack(self):
        tupl = (int(self.ready),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.ready = bool(fields[0]) 

class SetSubswarm(Message):
    msg_type = 0x89
    msg_fmt = '>B3x'

    def _init_message(self):
        self.subswarm = None         # New subswarm ID
        # 3 padding bytes = 0x00

    def _pack(self):
        tupl = (int(self.subswarm),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.subswarm = int(fields[0]) 

class SetController(Message):
    msg_type = 0x8A
    msg_fmt = '>B3x'

    def _init_message(self):
        self.controller = None     # Numeric ID of controller type
        # 3 padding bytes

    def _pack(self):
        tupl = (int(self.controller),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.controller = int(fields[0]) 

class FollowerSetup(Message):
    msg_type = 0x8B
    msg_fmt = '>BBhhhh2x'

    def _init_message(self):
        self.leader_id = None       # ID of aircraft to follow
        self.alt_mode = None        # 0=absolute, 1=relative
        self.seq = None             # Task sequence number (optional but should increment)
        self.follow_range = None    # Distance behind leader (meters)
        self.offset_angle = None    # Relative bearing to leader (radians)
        self.control_alt = None     # Relative or absolute altitude (meters)
        # 2 padding bytes

    def _pack(self):
        tupl = (self.leader_id,
                self.alt_mode,
                self.seq,
                int(self.follow_range),
                int(self.offset_angle * 1e3),
                int(self.control_alt))
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.leader_id = fields[0]
        self.alt_mode = fields[1]
        self.seq = fields[2]
        self.follow_range = float(fields[3])
        self.offset_angle = float(fields[4]) / 1e3
        self.control_alt = float(fields[5])

class WPSequencerSetup(Message):
    msg_type = 0x8C
    msg_fmt_base = '>BxH'
    msg_fmt_base_sz = struct.calcsize(msg_fmt_base)
    msg_fmt_wp = '>lll'
    msg_fmt_wp_sz = struct.calcsize(msg_fmt_wp)

    def _init_message(self):
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
        fmt = type(self).msg_fmt_base + \
              len(self.wp_list) * type(self).msg_fmt_wp.lstrip('>')
        return struct.pack(fmt, *tupl)

    def _unpack(self, data):
        (wp_count, self.seq) = struct.unpack_from(type(self).msg_fmt_base, data, 0)
        offset = type(self).msg_fmt_base_sz
        for wp in range(wp_count):
            lla = struct.unpack_from(type(self).msg_fmt_wp, data, offset)
            self.wp_list.append(lla)
            offset += type(self).msg_fmt_wp_sz

class PayloadHeartbeat(Message):
    msg_type = 0xFE
    msg_fmt = '>B3x'

    def _init_message(self):
        self.enable = None         # Boolean
        # 3 padding bytes = 0x00

    def _pack(self):
        tupl = (int(self.enable),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.enable = bool(fields[0])

class PayloadShutdown(Message):
    msg_type = 0xFF

    def _init_message(self):
        pass

    def _pack(self):
        return ''

    def _unpack(self, data):
        pass

