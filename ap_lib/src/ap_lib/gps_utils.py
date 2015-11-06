#!/usr/bin/env python

# Utility functions for working with GPS data
# Most functions were copied directly from MAVProxy's mp_util.y

from ap_lib import quaternion_math as qmath

import math
import os

EARTH_RADIUS = 6378100.0 # Approximate equatorial radius of the earth in meters

def gps_distance(lat1, lon1, lat2, lon2):
    '''return distance between two points in meters,
    coordinates are in degrees
    thanks to http://www.movable-type.co.uk/scripts/latlong.html'''
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    lon1 = math.radians(lon1)
    lon2 = math.radians(lon2)
    dLat = lat2 - lat1
    dLon = lon2 - lon1

    a = math.sin(0.5*dLat)**2 + math.sin(0.5*dLon)**2 * math.cos(lat1) * math.cos(lat2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0-a))
    return EARTH_RADIUS * c


def gps_bearing(lat1, lon1, lat2, lon2):
    '''return bearing between two points in radians, in range 0-2PI
    thanks to http://www.movable-type.co.uk/scripts/latlong.html
    Modified from original code to return radians vice degrees (dtd) '''
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    lon1 = math.radians(lon1)
    lon2 = math.radians(lon2)
    dLat = lat2 - lat1
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon)
    bearing = math.atan2(y, x)
    if bearing < 0:
        bearing += (math.pi * 2.0)
    return bearing


def wrap_valid_longitude(lon):
    ''' wrap a longitude value around to always have a value in the range
        [-180, +180) i.e 0 => 0, 1 => 1, -1 => -1, 181 => -179, -181 => 179
    '''
    return (((lon + 180.0) % 360.0) - 180.0)


def gps_newpos(lat, lon, bearing, distance):
    '''extrapolate latitude/longitude given a heading and distance
    thanks to http://www.movable-type.co.uk/scripts/latlong.html
    Modified from original code to work with radian bearing vice degrees (dtd)
    '''
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    brng = bearing
    dr = distance/EARTH_RADIUS

    lat2 = math.asin(math.sin(lat1)*math.cos(dr) +
                     math.cos(lat1)*math.sin(dr)*math.cos(brng))
    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(dr)*math.cos(lat1),
                             math.cos(dr)-math.sin(lat1)*math.sin(lat2))
    return (math.degrees(lat2), wrap_valid_longitude(math.degrees(lon2)))


def gps_offset(lat, lon, east, north):
    '''return new lat/lon after moving east/north
    by the given number of meters'''
    bearing = math.atan2(east, north)
    distance = math.sqrt(east**2 + north**2)
    return gps_newpos(lat, lon, bearing, distance)


def radius_at_latitude(lat):
    '''return the approximate radius of the earth at a specific latitude'''
    return EARTH_RADIUS * math.cos(math.radians(lat))


def normalize_angle(angle, radians = True):
    """normalizes an angle to the range ( -180, 180 ] """
    if not radians:
        angle = math.radians(angle)

    while angle > math.pi: angle -= (2.0 * math.pi)
    while angle <= -math.pi: angle += (2.0 * math.pi)

    if radians: return angle
    return math.degrees(angle)

#Returns True if a target position is "hit-able" by weapons at lat,lon,alt
#Returns False otherwise
#
#FOV_width and FOV_height specified in degrees.
def hitable(lat, lon, alt, pose_quat, max_range, FOV_width, FOV_height,
            targ_lat, targ_lon, targ_alt):
    # Compute lateral distance to leader (using lat/lon tools)
    distance = gps_distance( lat, lon, targ_lat, targ_lon )

    if (distance > max_range or distance <= 0):
        return False

    # Compute altitude difference to target
    #  - negative sign to indicate FROM me TO target
    alt_diff = -(alt - targ_alt)
    #with this implemenation: alt_diff / distance must be within the domain
    #of the arcsin function [-1, 1]:
    if (math.fabs(alt_diff) > distance):
        return False

    # Compute planar bearing from target
    # - measured in bearing angle (NED), radians
    abs_bearing = gps_bearing(lat, lon, targ_lat, targ_lon )

    rpy = qmath.quat_to_euler(pose_quat)
    yaw = rpy[2]

    #transform yaw to desired coord space
    if yaw < math.radians(180.0):
        yaw += math.radians(360.0)

    # Transform to account for current heading
    bearing = abs_bearing - yaw
    if bearing < 0.0:
        bearing += math.radians(360.0)

    # Check if meets FOV conditions
    #  -- within azimuthal scope (+/-FOV_half_width degrees)
    #  -- within elevation scope (+/-FOV_half_width degrees)
    FOV_half_width = math.radians(FOV_width / 2.0)
    if math.fabs(bearing) > FOV_half_width:
        return False

    # Compute and compare elevation
    FOV_half_height = math.radians(FOV_height / 2.0)
    phi = math.asin(alt_diff / distance)
    pitch = rpy[1]
    
    # Account for my pitch
    if alt_diff > 0.0:
        phi -= pitch
    else:
        phi += pitch

    if math.fabs(phi) > FOV_half_height:
        return False

    #all checks passed, target is 'shootable'
    return True

