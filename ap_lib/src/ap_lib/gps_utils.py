#!/usr/bin/env python

# Utility functions for working with GPS data
# Most functions were copied directly from MAVProxy's mp_util.y

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
