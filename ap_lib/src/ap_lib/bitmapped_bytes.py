#!/usr/bin/env python

#-------------------------------------------------------------------------
# bitmapped_bytes
# Duane Davis, 2015
#
# Abstract class to facilitate the development of behavior-specific
# parsers for the behavior parameter message generic byte arrays
#-------------------------------------------------------------------------

import struct
import ap_lib.acs_messages as acsmsg

# Enumeration for use in identifying message subtypes
PASSED_SHORT = 0    # Data field contains an short from a specific UAV
PASSED_USHORT = 1   # Data field contains an unsigned short from a specific UAV
PPASSED_INT = 2     # Data field contains an int from a specific UAV
PASSED_UINT = 3     # Data field contains an unsigned int from a specific UAV
PASSED_FLOAT = 4    # Data field contains a float from a specific UAV
SHORT_LIST = 5      # Data field contains a series of signed shorts
USHORT_LIST = 6     # Data field contains a series of unsigned shorts
INT_LIST = 7        # Data field contains a series of ints
UINT_LIST = 8       # Data field contains a series of unsigned ints
FLOAT_LIST = 9      # Data field contains a series of floats

SEARCH_WP = 10      # Data field represents a series of waypoints
FIRING_REPORT = 11  # Data field represents an air-to-air firing report
ID_VALUE_PAIRS = 12 # Data field represents a series of ID-value pairs
CONSENSUS_SUMMARY = 13 # Data field represents a consensus algorithm summary

class BitmappedBytes(object):
    ''' Abstract class template for customized parsing of byte arrays
    Implementing classes should add member variables as required and implement
    the virtual "pack" and "serialize" methods.  Once implemented, objects
    will be able to set custom parameter values by im

    Member variables:
      format: field desribing byte array contents (set by implementing class)

    Class methods:
      acs_message: creates an ACS network SwarmBehavior message for the object

    "Virtual" methods for inheriting class implementation
      pack: generates a byte array corresponding to the class parameters
      unpack: uses a byte array to set implementing class parameter values
    '''

    #---------------------------------------------------------------------
    # "Virtual" methods of this class (must be implement in child classes)
    #---------------------------------------------------------------------

    def pack(self):
        ''' Converts parameter field to a byte array
        @return: a byte array of bitmap-encoded parameter values
        '''
        return ""


    def unpack(self, bytes):
        ''' Sets parameter values based on a bitmapped byte array
        @param bytes: bitmap-encoded parameter values byte array
        @return True if the byte
        '''
        pass


class PassedValueParser(BitmappedBytes):
    ''' Abstract class for values passed from a particular source
        Can be extended as required to pass different value types
    '''
    fmt_base = ">B3x"
    fmt_base_sz = struct.calcsize(fmt_base)
    data_fmt = ''

    def __init(self):
        ''' Initializes parameters with default values
        '''
        self.source = 0
        self.value = 0


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        tupl = (self.source, self.value)
        fmt = type(self).fmt_base + type(self).data_fmt
        return struct.pack(fmt, *tupl)


    def unpack(self, bytes):
        ''' Sets parameter values from a bitmapped byte array
        @param bytes: bitmapped byte array
        '''
        fmt = type(self).fmt_base + type(self).data_fmt
        self.source, self.value = struct.unpack_from(fmt, bytes)


class VariableLengthBitmappedBytes(BitmappedBytes):
    ''' Parent class for a variable length bitmapped byte message
    The format includes an unsigned byte field for the source of the message
    (optional, but the value always needs to be parsed) and number of elements.
    '''
    fmt_base = ">BB2x"
    fmt_base_sz = struct.calcsize(fmt_base)


class NumberListParser(VariableLengthBitmappedBytes):
    ''' Parser for data string of single-type numerical values
        This class can be extended for arbitrary numerical types
    ''' 
    data_fmt = ''

    def __init__(self):
        ''' Initializes parameters with default values
        '''
        self.source_id = 0  # Source UAV of the data
        self.number_list = [] # list of numerical values


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        num_values = len(self.number_list)
        tupl = (self.source_id, num_values)
        for value_num in range(num_values):
            tupl += (self.number_list[value_num],)
        fmt = type(self).fmt_base + num_values * type(self).data_fmt
        return struct.pack(fmt, *tupl)


    def unpack(self, bytes):
        ''' Sets parameter values from a bitmapped byte array
        @param bytes: bitmapped byte array
        '''
        (self.source_id, num_values) = \
            struct.unpack_from(type(self).fmt_base, bytes, 0)
        fmt = type(self).fmt_base + num_values * type(self).data_fmt
        self.number_list = struct.unpack_from(fmt, bytes)[2:]


#-----------------------------
# Inheriting class definitions
#-----------------------------

# A few generic parameter passing parsers that might be broadly useful

class PassedShortParser(PassedValueParser):
    ''' Parser for a signed short value passed from a specific UAV
    '''
    data_fmt = 'h'


class PassedUShortParser(PassedValueParser):
    ''' Parser for an unsigned short value passed from a specific UAV
    '''
    data_fmt = 'H'


class PassedIntParser(PassedValueParser):
    ''' Parser for a signed int value passed from a specific UAV
    '''
    data_fmt = 'i'


class PassedUIntParser(PassedValueParser):
    ''' Parser for an unsigned int value passed from a specific UAV
    '''
    data_fmt = 'I'


class PassedFloatParser(PassedValueParser):
    ''' Parser for a float value passed from a specific UAV
    '''
    data_fmt = 'f'


# A few generic variable length parsers that might be broadly useful

class ShortListParser(NumberListParser):
    ''' Parser for data lines consisting of a series of signed short ints
    '''
    data_fmt = 'h'


class UShortListParser(NumberListParser):
    ''' Parser for data lines consisting of a series of unsigned short ints
    '''
    data_fmt = 'H'


class IntListParser(NumberListParser):
    ''' Parser for data lines consisting of a series of signed ints
    '''
    data_fmt = 'i'


class UIntListParser(NumberListParser):
    ''' Parser for data lines consisting of a series of unsigned ints
    '''
    data_fmt = 'I'


class FloatListParser(NumberListParser):
    ''' Parser for data lines consisting of a series of floats
    '''
    data_fmt = 'f'


class IdShortValuePairParser(VariableLengthBitmappedBytes):
    ''' Parser for a series of UAV/value pair tuples (unsigned short value)
    '''
    fmt_pair = ">BxH"
    fmt_pair_sz = struct.calcsize(fmt_pair)

    def __init__(self):
        ''' Initializes parameters with default values
        '''
        self.source_id = 0  # Source UAV of the data
        self.pairs = [] # list of (ID, value) pairs


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        num_pairs = len(self.pairs)
        tupl = (self.source_id, num_pairs)
        for pair_num in range(num_pairs):
            tupl += ( int(self.pairs[pair_num][0]), \
                      int(self.pairs[pair_num][1]) )
        fmt = type(self).fmt_base + num_pairs * type(self).fmt_pair.lstrip('>')
        return struct.pack(fmt, *tupl)


    def unpack(self, bytes):
        ''' Sets parameter values from a bitmapped byte array
        @param bytes: bitmapped byte array
        '''
        (self.source_id, num_pairs) = \
            struct.unpack_from(type(self).fmt_base, bytes, 0)
        self.pairs = []
        offset = type(self).fmt_base_sz
        for pair_num in range(num_pairs):
            fields = struct.unpack_from(type(self).fmt_pair, bytes, offset)
            self.pairs.append((fields[0], fields[1]))
            offset += type(self).fmt_pair_sz


# Parsers corresponding to specific message types

class SearchOrderParser(BitmappedBytes):
    ''' Parser swarm search orders
    '''
    fmt = ">ffffBB"

    def __init__(self):
        ''' Initializes parameters with default values
        '''
        self.lat = 0.0
        self.lon = 0.0
        self.areaLength = 0.0
        self.areaWidth = 0.0
        self.masterID = 0
        self.algorithmNumber = 0


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        return struct.pack(type(self).fmt, self.lat, self.lon, \
                           self.areaLength, self.areaWidth, \
                           self.masterID, self.algorithmNumber)


    def unpack(self, bytes):
        ''' Sets parameter values from a bitmapped byte array
        @param bytes: bitmapped byte array
        '''
        self.lat, self.lon, self.areaLength, self.areaWidth, \
            self.masterID, self.algorithmNumber = \
                struct.unpack_from(type(self).fmt, bytes, 0)


class SearchWaypointParser(VariableLengthBitmappedBytes):
    ''' Parser for swarm search waypoint messages
    '''
    fmt_wp = '>lllBBBx'
    fmt_wp_sz = struct.calcsize(fmt_wp)


    def __init__(self):
        ''' Initializes parameters with default values
        '''
        self.source_id = 0  # Source UAV of the data
        self.wp_list = [] # WP format: (lat, lon, rel_alt, vid, cell_x, cell_y)


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        tupl = (self.source_id, len(self.wp_list))
        for wp in self.wp_list:
            tupl += (int(wp[0] * 1e07), # Degrees * 1e7
                     int(wp[1] * 1e07), # Degrees * 1e7
                     int(wp[2] * 1e03), # rel_alt * 1000 
                     int(wp[3]),  # Vehicle ID (0-255)
                     int(wp[4]),  # Cell X (0-255)
                     int(wp[5]))  # Cell Y (0-255)
        fmt = type(self).fmt_base + \
              len(self.wp_list) * type(self).fmt_wp.lstrip('>')
        return struct.pack(fmt, *tupl)


    def unpack(self, bytes):
        ''' Sets parameter values from a bitmapped byte array
        @param bytes: bitmapped byte array
        '''
        (self.source_id, num_wps) = \
            struct.unpack_from(type(self).fmt_base, bytes, 0)
        self.wp_list = []
        offset = type(self).fmt_base_sz
        for wp_num in range(num_wps):
            fields = struct.unpack_from(type(self).fmt_wp, bytes, offset)
            self.wp_list. \
                 append((fields[0] / 1e07, fields[1] / 1e07, fields[2] / 1e03,\
                         fields[3], fields[4], fields[5]))
            offset += type(self).fmt_wp_sz


class LandingOrderParser(BitmappedBytes):
    ''' Parser for landing orders
    '''
    fmt = ">B"

    def __init__(self):
        ''' Initializes parameters with default values
        '''
        self.landing_wp_id = 0


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        return struct.pack(type(self).fmt, self.landing_wp_id)


    def unpack(self, bytes):
        ''' Sets parameter values from a bitmapped byte array
        @param bytes: bitmapped byte array
        '''
        self.landing_wp_id, = struct.unpack_from(type(self).fmt, bytes, 0)


class LinearFormationOrderParser(BitmappedBytes):
    ''' Parser for linear formation orders
    '''
    fmt = ">ff?"

    def __init__(self):
        ''' Initializes parameters with default values
        '''
        self.distance = 0.0
        self.angle = 0.0
        self.stack_formation = True


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        return struct.pack(type(self).fmt, self.distance, \
                           self.angle, self.stack_formation)


    def unpack(self, bytes):
        ''' Sets parameter values from a bitmapped byte array
        @param bytes: bitmapped byte array
        '''
        self.distance, self.angle, self.stack_formation = \
            struct.unpack_from(type(self).fmt, bytes, 0)


class FiringReportParser(BitmappedBytes):
    ''' Parser for firing report messages orders
    '''
    fmt = ">BlllllhBlll"

    def __init__(self):
        ''' Initializes parameters with default values
        '''
        self.report_num = 0
        self.time_secs = 0
        self.time_nsecs = 0
        self.lat = 0.0         # 7-decimal precision degrees
        self.lon = 0.0         # 7-decimal precision degrees
        self.alt = 0.0         # 2-decimal precision meters MSL
        self.heading = 0.0     # 3-decimal precision radians
        self.target_id = 0     # 0-255
        self.target_lat = 0.0  # 7-decimal precision degrees
        self.target_lon = 0.0  # 7-decimal precision degrees
        self.target_alt = 0.0  # 2-decimal precision meters MSL


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        tupl = (self.report_num, self.time_secs, self.time_nsecs, \
                int(self.lat * 1e07), \
                int(self.lon * 1e07), \
                int(self.alt * 1e02), \
                int(self.heading * 1e03), \
                self.target_id, \
                int(self.target_lat * 1e07), \
                int(self.target_lon * 1e07), \
                int(self.target_alt * 1e02))

        return struct.pack(type(self).fmt, *tupl)


    def unpack(self, bytes):
        ''' Sets parameter values from a bitmapped byte array
        @param bytes: bitmapped byte array
        '''
        fields = struct.unpack_from(type(self).fmt, bytes, 0)

        # Place unpacked but unconverted fields into message elements
        self.report_num = fields[0]
        self.time_secs = fields[1]
        self.time_nsecs = fields[2]
        self.lat = fields[3] / 1e07
        self.lon = fields[4] / 1e07
        self.alt = fields[5] / 1e02
        self.heading = fields[6] / 1e03
        self.target_id = fields[7]
        self.target_lat = fields[8] / 1e07
        self.target_lon = fields[9] / 1e07
        self.target_alt = fields[10] / 1e02


class ConsensusSummaryParser(BitmappedBytes):
    ''' Utility message to transmit consensus algorithm summary data
    '''
    fmt='>HHH2xL'

    def __init__(self):
        ''' Initializes parameters with default values
        '''
        self.algorithm = 0
        self.rounds = 0
        self.messages = 0
        self.bytes = 0


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        tupl = (self.algorithm, self.rounds, self.messages, self.bytes)
        return struct.pack(type(self).fmt, *tupl)


    def unpack(self, bytes):
        ''' Sets parameter values from a bitmapped byte array
        @param bytes: bitmapped byte array
        '''
        self.algorithm, self.rounds, self.messages, self.bytes = \
            struct.unpack_from(type(self).fmt, bytes, 0)

