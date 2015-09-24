#!/usr/bin/env python
#
#Utility to create imagergy from ROS bags from ACS flights.
#
#Depends heavily on MAVLink (pymavlink) and MAVProxy utilities.
#
#Author: Michael Day
#Date: Sept 2015

import sys, time, os

import rosbag

from pymavlink import mavutil, mavwp, mavextra
from MAVProxy.modules.mavproxy_map import mp_slipmap, mp_tile
from MAVProxy.modules.lib import mp_util
import functools

try:
    import cv2.cv as cv
except ImportError:
    import cv

from optparse import OptionParser
parser = OptionParser("bag_flight_view.py [options] BAGFILE1 <BAGFILE2 ...>")
parser.add_option("--service", default="MicrosoftSat", help="tile service")
parser.add_option("--fence", default=None, help="fence file")
parser.add_option("--imagefile", default=None, help="output to image file")
parser.add_option("--debug", action='store_true', default=False, help="show debug info")
parser.add_option("--multi", action='store_true', default=False, help="show multiple flights on one map")
parser.add_option("--mintime", type='int', default=0, help="Minimum time stamp value for output file(s)")
parser.add_option("--maxtime", type='int', default=0, help="Maximum time stamp value for output file(s)")
parser.add_option("--rate", type='int', default=0, help="maximum message rate to display (0 means all points)")
parser.add_option("--flag", default=[], type='str', action='append', help="flag positions")
parser.add_option("--dir", default=None, 
    help="directory mode (create imgs for all bags in given dir)")

(opts, args) = parser.parse_args()

def pixel_coords(latlon, ground_width=0, mt=None, topleft=None, width=None):
    '''return pixel coordinates in the map image for a (lat,lon)'''
    (lat,lon) = (latlon[0], latlon[1])
    return mt.coord_to_pixel(topleft[0], topleft[1], width, ground_width, lat, lon)

def create_imagefile(filename, latlon, ground_width, path_objs, mission_obj, fence_obj, width=600, height=600):
    '''create path and mission as an image file'''
    mt = mp_tile.MPTile(service=opts.service)

    map_img = mt.area_to_image(latlon[0], latlon[1],
                               width, height, ground_width)
    while mt.tiles_pending() > 0:
        print("Waiting on %u tiles" % mt.tiles_pending())
        time.sleep(1)
    map_img = mt.area_to_image(latlon[0], latlon[1],
                               width, height, ground_width)
    # a function to convert from (lat,lon) to (px,py) on the map
    pixmapper = functools.partial(pixel_coords, ground_width=ground_width, mt=mt, topleft=latlon, width=width)
    for path_obj in path_objs:
        path_obj.draw(map_img, pixmapper, None)
    if mission_obj is not None:
        for m in mission_obj:
            m.draw(map_img, pixmapper, None)
    if fence_obj is not None:
        fence_obj.draw(map_img, pixmapper, None)
    cv.CvtColor(map_img, map_img, cv.CV_BGR2RGB)
    cv.SaveImage(filename, map_img)

def bagflightview(filename):
    print("Loading %s ..." % filename)

    bag = rosbag.Bag(filename)

    fen = mavwp.MAVFenceLoader()
    if opts.fence is not None:
        fen.load(opts.fence)
        
    path = []
    last_timestamp = 0

    for topic, msg, t in bag.read_messages(topics=['/autopilot/acs_pose']):
        tim = float(str(t)) / 1e9

        if opts.mintime != 0 and tim < opts.mintime:
            continue
        if opts.maxtime != 0 and tim > opts.maxtime:
            continue

        lat = msg.pose.pose.position.lat
        lng = msg.pose.pose.position.lon
        #alt = msg.pose.pose.position.alt
        
        #TODO: May want to check GPS "status" in the future at the payload...
        #if type in ['GPS','GPS2']:
        #    status = getattr(m, 'Status', None)
        #    if status is None:
        #        status = getattr(m, 'FixType', None)
        #        if status is None:
        #            print("Can't find status on GPS message")
        #            print(m)
        #            break
        #    if status < 2:
        #        continue

        #TODO: maybe color by mode someday:
        colour = (0, 0, 255)
        point = (lat, lng, colour)

        if opts.rate == 0 or tim - last_timestamp > 1.0/opts.rate:
            last_timestamp = tim
            path.append(point)

    if len(path) == 0:
        print("No points to plot")
        return

    bounds = mp_util.polygon_bounds(path)
    (lat, lon) = (bounds[0]+bounds[2], bounds[1])
    (lat, lon) = mp_util.gps_newpos(lat, lon, -45, 50)
    ground_width = mp_util.gps_distance(lat, lon, lat-bounds[2], lon+bounds[3])
    while (mp_util.gps_distance(lat, lon, bounds[0], bounds[1]) >= ground_width-20 or
           mp_util.gps_distance(lat, lon, lat, bounds[1]+bounds[3]) >= ground_width-20):
        ground_width += 10

    path_objs = []
    
    path_objs.append(mp_slipmap.SlipPolygon('FlightPath[%u]-%s' % (0,filename), path, layer='FlightPath', linewidth=2, colour=(255,0,180)))

    #Not doing waypoints
    mission_obj = None

    fence = fen.polygon()
    if len(fence) > 1:
        fence_obj = mp_slipmap.SlipPolygon('Fence-%s' % filename, fen.polygon(), layer='Fence', linewidth=2, colour=(0,255,0))
    else:
        fence_obj = None

    if opts.imagefile:
        create_imagefile(opts.imagefile, (lat,lon), ground_width, path_objs, mission_obj, fence_obj)
    elif opts.dir is not None:
        path_tokens = os.path.split(filename)
        file_name, ext = os.path.splitext(path_tokens[-1])
        
        create_imagefile(filename + ".png", (lat,lon), ground_width, path_objs, mission_obj, fence_obj)
    else:
        global multi_map
        if opts.multi and multi_map is not None:
            map = multi_map
        else:
            map = mp_slipmap.MPSlipMap(title=filename,
                                       service=opts.service,
                                       elevation=True,
                                       width=600,
                                       height=600,
                                       ground_width=ground_width,
                                       lat=lat, lon=lon,
                                       debug=opts.debug)
        if opts.multi:
            multi_map = map
        for path_obj in path_objs:
            map.add_object(path_obj)
        if mission_obj is not None:
            display_waypoints(wp, map)
        if fence_obj is not None:
            map.add_object(fence_obj)

        for flag in opts.flag:
            a = flag.split(',')
            lat = a[0]
            lon = a[1]
            icon = 'flag.png'
            if len(a) > 2:
                icon = a[2] + '.png'
            icon = map.icon(icon)
            map.add_object(mp_slipmap.SlipIcon('icon - %s' % str(flag), (float(lat),float(lon)), icon, layer=3, rotation=0, follow=False))

if opts.dir is None and len(args) < 1:
    parser.print_usage()
    sys.exit(1)

if __name__ == "__main__":
    if opts.dir is not None:
        #directory mode means create images for all bag files in a directory
        for current_dir, dirs, files in os.walk(opts.dir):
            print("Processing directory " + current_dir)
            for f in files:
                file_name, ext = os.path.splitext(f)
                fullpath = os.path.join(current_dir,f)

                if ext.lower() == ".bag":
                    bagflightview(fullpath)

    else:
        if opts.multi:
            multi_map = None

        for f in args:
            bagflightview(f)
