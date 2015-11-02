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


#-----------------------------
# Inheriting class definitions
#-----------------------------

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


class SearchWaypointParser(BitmappedBytes):
    ''' Parser for swarm search waypoint messages
    '''
    fmt_base = ">B3x"
    fmt_base_sz = struct.calcsize(fmt_base)
    fmt_wp = '>lllBBBx'
    fmt_wp_size = struct.calcsize(fmt_wp)

    SEARCH_WP = 0   # Enumeration for use in specifying message type

    def __init__(self):
        ''' Initializes parameters with default values
        '''
        self.wp_list = [] # WP format: (lat, lon, rel_alt, vid, cell_x, cell_y)


    def pack(self):
        ''' Serializes parameter values into a bitmapped byte array
        @return bitmapped bytes as a string
        '''
        tupl = (len(self.wp_list),)
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
        (num_wps,) = struct.unpack_from(type(self).fmt_base, bytes, 0)
        self.wp_list = []
        offset = type(self).fmt_base_sz
        for wp_num in range(num_wps):
            fields = struct.unpack_from(type(self).fmt_wp, bytes, offset)
            self.wp_list. \
                 append((fields[0] / 1e07, fields[1] / 1e07, fields[2] / 1e03,\
                         fields[3], fields[4], fields[5]))
            offset += type(self).fmt_wp_size


class LandingOrderParser(BitmappedBytes):
    ''' Parser for landing orders
    '''
    fmt = "B"

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

