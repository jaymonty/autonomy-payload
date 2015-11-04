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

# Python2/3 string/bytes compatibility
def _enc_str(s):
    try:
        return s.encode('utf-8')  # Python 3
    except:
        return str(s)  # Python 2
def _dec_str(e):
    try:
        s = e.decode('utf-8')
    except:
        s = str(e)
    finally:
        return s.strip(chr(0x0))

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
        # Initialize message size
        self.msg_size = self.hdr_size + struct.calcsize(self.msg_fmt)

        # Initialize header fields
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
        payload = self._pack()
        if payload != '':
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
            raise Exception("bad header: " + str(ex))

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
            raise Exception("bad payload: " + str(ex))

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

    def __init__(self):
        Message.__init__(self)

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
    msg_fmt = '>HBBHhhHB3x16s'

    def __init__(self):
        Message.__init__(self)

        # Define message fields (setting to None helps raise Exceptions later)
        # All of these are in the first 'H' (mode_and_flags)
        self.mode = None	# Aircraft guidance mode (0-15, see enum)
        self.armed = None	# Boolean: Throttle Armed?
        self.ok_ahrs = None	# Boolean: AHRS OK?
        self.ok_as = None	# Boolean: Airspeed Sensor OK?
        self.ok_gps = None	# Boolean: GPS sensor OK?
        self.ok_ins = None	# Boolean: INS sensor OK?
        self.ok_mag = None	# Boolean: Magnetometer OK?
        self.ok_pwr = None	# Boolean: Power OK?
        self.ready = None	# Boolean: Ready for flight? (user-settable)
        self.ok_prm = None  # Boolean: Params verified?
        self.ok_fen = None  # Boolean: Fence verified?
        self.ok_ral = None  # Boolean: Rally verified?
        self.ok_wp = None   # Boolean: Waypoints verified?
        # Next byte: 4 bits (swarm state), 2 bits (fence), 2 bits UNUSED
        self.swarm_state = 0    # Swarm state (0 = preflight)
        self.fence_state = 2    # Fence state (2 = disabled)
        # These are the remainder, starting with the second 'B'
        self.batt_rem = None	# Battery % remaining (int, 0-100)
        self.batt_vcc = None	# Battery Voltage (int, mV)
        self.airspeed = None	# Airspeed (float, m/s)
        self.alt_rel = None	    # AGL (int, millimeters)
        self.mis_cur = None	    # Current mission (waypoint) index (0-65535)
        self.swarm_behavior = 0 # Swarm behavior (follow, standby, etc.)
        # Next 3 bytes are unused
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
                       | (0x0008 & _bool16(self.ok_prm)) \
                       | (0x0004 & _bool16(self.ok_fen)) \
                       | (0x0002 & _bool16(self.ok_ral)) \
                       | (0x0001 & _bool16(self.ok_wp)) \
                       & 0xffff  # Zeroize any unused bits
        swarm_fence = ((self.swarm_state & 0x0F) << 4) \
                    | ((self.fence_state & 0x03) << 2) \
                    & 0xfc  # Zeroize any unused bits
        batt_rem = 255
        if 0 <= self.batt_rem <= 100:  # Set invalid values to max unsigned
            batt_rem = self.batt_rem
        batt_vcc = 65535
        if self.batt_vcc >= 0:
            batt_vcc = self.batt_vcc
        tupl = (mode_and_flags,
                swarm_fence,
                int(batt_rem),
                int(batt_vcc),
                int(self.airspeed * 1e02),  # TODO: Are these large enough?
                int(self.alt_rel / 1e02),
                int(self.mis_cur),
                int(self.swarm_behavior),
                _enc_str(self.name))

        # Pack into a byte string
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        # Unpack payload into fields
        fields = list(struct.unpack_from(type(self).msg_fmt, data, 0))

        # Place unpacked but unconverted fields into message elements
        modeflag = fields.pop(0)
        self.mode = modeflag >> 12
        self.armed = bool(modeflag & 0x0800)
        self.ok_ahrs = bool(modeflag & 0x0400)
        self.ok_as = bool(modeflag & 0x0200)
        self.ok_gps = bool(modeflag & 0x0100)
        self.ok_ins = bool(modeflag & 0x0080)
        self.ok_mag = bool(modeflag & 0x0040)
        self.ok_pwr = bool(modeflag & 0x0020)
        self.ready = bool(modeflag & 0x0010)
        self.ok_prm = bool(modeflag & 0x0008)
        self.ok_fen = bool(modeflag & 0x0004)
        self.ok_ral = bool(modeflag & 0x0002)
        self.ok_wp = bool(modeflag & 0x0001)
        swarmfence = fields.pop(0)
        self.swarm_state = int((swarmfence >> 4) & 0x0F)
        self.fence_state = int((swarmfence >> 2) & 0x03)
        self.batt_rem = fields.pop(0)
        if self.batt_rem == 255:  # Account for invalid values
            self.batt_rem = -1
        self.batt_vcc = fields.pop(0)
        if self.batt_vcc == 65535:  # Account for invalid values
            self.batt_vcc = -1
        self.airspeed = fields.pop(0) / 1e02
        self.alt_rel = fields.pop(0) * 1e02
        self.mis_cur = fields.pop(0)
        self.swarm_behavior = fields.pop(0)
        self.name = _dec_str(fields.pop(0))

class Pose(Message):
    # Define message type parameters
    msg_type = 0x01
    msg_fmt = '>lllllllhhhhhh'

    def __init__(self):
        Message.__init__(self)

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

class VehicleIntent(Message):
    msg_type = 0x02
    msg_fmt = '>blll'

    def __init__(self):
        Message.__init__(self)

        self.swarm_behavior = None # Swarm Behavior publishing this intent
        self.lat = None		# Decimal degrees (e.g. 35.123456)
        self.lon = None		# Decimal degrees (e.g. -120.123456)
        self.alt = None		# Decimal meters MSL (WGS84)

    def _pack(self):
        tupl = (int(self.swarm_behavior),
                int(self.lat * 1e07),
                int(self.lon * 1e07),
                int(self.alt * 1e03))
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.swarm_behavior = int(fields[0])
        self.lat = fields[1] / 1e07
        self.lon = fields[2] / 1e07
        self.alt = fields[3] / 1e03

class Heartbeat(Message):
    msg_type = 0x80
    msg_fmt = '>L'

    def __init__(self):
        Message.__init__(self)

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

    def __init__(self):
        Message.__init__(self)

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

    def __init__(self):
        Message.__init__(self)

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

    def __init__(self):
        Message.__init__(self)

        pass

    def _pack(self):
        return ''

    def _unpack(self, data):
        pass

class LandAbort(Message):
    msg_type = 0x84
    msg_fmt = '>h2x'

    def __init__(self):
        Message.__init__(self)

        self.alt = None         # Wave-off altitude (approx +/-32000)
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

    def __init__(self):
        Message.__init__(self)

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

    def __init__(self):
        Message.__init__(self)

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

    def __init__(self):
        Message.__init__(self)

        self.enable = None         # Boolean
        self.channel = None        # Pascal String

    def _pack(self):
        # TODO Get rid of this ugly hack
        try:
            channel = bytes(self.channel, 'utf-8')  # Python3
        except:
            channel = str(self.channel)  # Python2

        tupl = (int(self.enable),
                _enc_str(channel))
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.enable = bool(fields[0])
        self.channel = _dec_str(fields[1])

class FlightReady(Message):
    msg_type = 0x88
    msg_fmt = '>B3x'

    def __init__(self):
        Message.__init__(self)

        self.ready = None         # Boolean
        # 3 padding bytes = 0x00

    def _pack(self):
        tupl = (int(self.ready),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.ready = bool(fields[0])

class Calibrate(Message):
    msg_type = 0x8D
    msg_fmt = '>B3x'

    def __init__(self):
        Message.__init__(self)

        self.index = None   # Numeric ID of calibration
        # 3 bytes of padding

    def _pack(self):
        tupl = (int(self.index),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.index = int(fields[0])

class Demo(Message):
    msg_type = 0x90
    msg_fmt = '>B3x'

    def __init__(self):
        Message.__init__(self)

        self.demo = None     # Numeric ID of demo
        # 3 padding bytes

    def _pack(self):
        tupl = (int(self.demo),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.demo = int(fields[0])

class MissionConfig(Message):
    msg_type = 0x91
    msg_fmt = '>HBx'

    def __init__(self):
        Message.__init__(self)

        self.std_alt = None     # Standard (RELATIVE) altitude (m)
        self.stack_num = None   # Index of stack (of pancakes) to be in
        # 1 padding byte

    def _pack(self):
        tupl = (int(self.std_alt), int(self.stack_num))
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.std_alt = int(fields[0])
        self.stack_num = int(fields[1])

class WeatherData(Message):
    msg_type = 0x92
    msg_fmt = '>ffff'

    def __init__(self):
        Message.__init__(self)

        self.baro = 0.0           # barametric pressure (millibars)
        self.temperature = 0.0    # temperature (degrees C)
        self.wind_speed = 0.0     # miles per hour (need to convert)
        self.wind_direction = 0.0 # direction from (degrees)

    def _pack(self):
        tupl = (float(self.baro), float(self.temperature), float(self.wind_speed), float(self.wind_direction))
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.baro = fields[0]
        self.temperature = fields[1]
        self.wind_speed = fields[2]
        self.wind_direction = fields[3]

#Message to request the previous N autopilot messages.
#A second field, since_seq, is intended to signal that the
#requestor already has up to that sequence number.  If that
#sequence number is arrived at before processing N autopilot
#messages, the request doesn't want the other msgs (already has them)
class ReqPrevNMsgsAP(Message):
    msg_type = 0xA0
    msg_fmt = '>LB3x'

    def __init__(self):
        Message.__init__(self)

        self.since_seq = 0
        self.n = 0

    def _pack(self):
        tupl = (int(self.since_seq), int(self.n))
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.since_seq = fields[0]
        self.n = fields[1]

#ACS Message containing a previous autopilot message.
class PrevMsgAP(Message):
    msg_type = 0xA1
    msg_fmt = '>LL92s'

    def __init__(self):
        Message.__init__(self)

        self.final_seq = 0  #Each message states the seq number at the
                            #end of the msg queue so the requestor knows
                            #how many messages are available, even if
                            #the requestor does not receive all messages
                            #in a given series of responses
        self.seq = 0
        self.msg = ""

    def _pack(self):
        tupl = (int(self.final_seq), int(self.seq), _enc_str(self.msg))
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.final_seq = fields[0]
        self.seq = fields[1]
        self.msg = _dec_str(fields[2])

class AutopilotReboot(Message):
    msg_type = 0xFD

    def __init__(self):
        Message.__init__(self)

        pass

    def _pack(self):
        return ''

    def _unpack(self, data):
        pass

class PayloadHeartbeat(Message):
    msg_type = 0xFE
    msg_fmt = '>B3x'

    def __init__(self):
        Message.__init__(self)

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

    def __init__(self):
        Message.__init__(self)

        pass

    def _pack(self):
        return ''

    def _unpack(self, data):
        pass

# Swarm and subswarm manipulation and control messages

class SetSubswarm(Message):
    ''' Assign the recipient vehicle to a specific subswarm '''
    msg_type = 0x70
    msg_fmt = '>B3x'

    def __init__(self):
        Message.__init__(self)

        self.subswarm = None         # New subswarm ID
        # 3 padding bytes = 0x00

    def _pack(self):
        tupl = (int(self.subswarm),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.subswarm = int(fields[0])

class SwarmState(Message):
    ''' Manually set the vehicle's swarm state '''
    msg_type = 0x71
    msg_fmt = '>B3x'

    def __init__(self):
        Message.__init__(self)

        self.swarm_state = 0  # swarm_state value

    def _pack(self):
        tupl = (int(self.swarm_state),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.swarm_state = int(fields[0])

class SwarmBehavior(Message):
    ''' Initiate execution of a specific parameterized swarm behavior '''
    msg_type = 0x72
    msg_fmt_base = '>BHx'
    msg_fmt_base_sz = struct.calcsize(msg_fmt_base)

    def __init__(self):
        Message.__init__(self)

        self.swarm_behavior = None
        self.swarm_parameters = []

    def _pack(self):
        self.swarm_parameters = \
            self.swarm_parameters + (b'\x00' * (len(self.swarm_parameters) % 4))
        param_bytes = len(self.swarm_parameters)
        tupl = (self.swarm_behavior, param_bytes)
        for byte in range(0, param_bytes):
            tupl += (self.swarm_parameters[byte],)
        fmt = type(self).msg_fmt_base + (param_bytes * 'B')
        return struct.pack(fmt, *tupl)

    def _unpack(self, data):
        (self.swarm_behavior, param_bytes) = \
            struct.unpack_from(type(self).msg_fmt_base, data, 0)
        offset = type(self).msg_fmt_base_sz
        self.swarm_parameters = data[offset:]

class SwarmBehaviorData(Message):
    ''' Exchange behavior-specific information between vehicles '''
    msg_type = 0x73
    msg_fmt_base = '>BHx'
    msg_fmt_base_sz = struct.calcsize(msg_fmt_base)

    def __init__(self):
        Message.__init__(self)

        self.data_type = None  # For any required payload disambiguation
        self.data = []         # Bitmapped byte string with message payload

    def _pack(self):
        self.data = \
            self.data + (b'\x00' * (len(self.data) % 4))
        data_bytes = len(self.data)
        tupl = (self.data_type, data_bytes)
        for byte in range(0, data_bytes):
            # Differs from the SwarmBehavior message because of Python 2 vs 3?
            # Encoded by the payload (Python 2) for this message
            # Encoded by SwarmCommander (Python 3) for this message
            tupl += (ord(self.data[byte]),)
        fmt = type(self).msg_fmt_base + (data_bytes * 'B')
        return struct.pack(fmt, *tupl)

    def _unpack(self, data):
        (self.data_type, data_bytes) = \
            struct.unpack_from(type(self).msg_fmt_base, data, 0)
        offset = type(self).msg_fmt_base_sz
        self.data = data[offset:]

class SuspendSwarmBehavior(Message):
    ''' Terminate execution of the current swarm behavior '''
    msg_type = 0x74

    def __init__(self):
        Message.__init__(self)
        pass

    def _pack(self):
        return ''

    def _unpack(self, data):
        pass

class PauseSwarmBehavior(Message):
    ''' Pause or resume execution of the current swarm behavior '''
    msg_type = 0x75
    msg_fmt = '>?3x'

    def __init__(self):
        Message.__init__(self)
        self.behavior_pause = None

    def _pack(self):
        tupl = (bool(self.behavior_pause),)
        return struct.pack(type(self).msg_fmt, *tupl)

    def _unpack(self, data):
        fields = struct.unpack_from(type(self).msg_fmt, data, 0)
        self.behavior_pause = bool(fields[0])

class RedPose(Message):
    ''' Pose data for a "red" UAV '''
    msg_type = 0x76
    msg_fmt = '>B3xlllllllhhhhhh'

    def __init__(self):
        Message.__init__(self)

        # Define message fields (setting to None helps raise Exceptions later)
        self.uav_id = None      # ID (int) of the red UAV
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
        tupl = (self.uav_id,
                int(self.lat * 1e07),
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
        self.uav_id = fields[0]
        self.lat = fields[1] / 1e07
        self.lon = fields[2] / 1e07
        self.alt = fields[3] / 1e03
        self.q_x = fields[4] / 1e09
        self.q_y = fields[5] / 1e09
        self.q_z = fields[6] / 1e09
        self.q_w = fields[7] / 1e09
        self.vlx = fields[8] / 1e02
        self.vly = fields[9] / 1e02
        self.vlz = fields[10] / 1e02
        self.vax = fields[11] / 1e02
        self.vay = fields[12] / 1e02
        self.vaz = fields[13] / 1e02
