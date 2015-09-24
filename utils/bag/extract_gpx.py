#!/usr/bin/env python
#
#Utility to create a GPX file from ROS bags from ACS flights.
#
#Author: Michael Day
#Date: Sept 2015

import rosbag
import tf
import math, os, sys, time

from ap_lib import gps_utils

from optparse import OptionParser
parser = OptionParser("extract_gpx.py [options] BAGFILE GPXFILE")
parser.add_option("--dir", default=None, 
    help="directory mode (create gpx files for all bags in given dir)")
parser.add_option("--mintime", type='int', default=0, help="Minimum time stamp value for output file(s)")
parser.add_option("--maxtime", type='int', default=0, help="Maximum time stamp value for output file(s)")

(opts, args) = parser.parse_args()

def extractgpx(filename, outfilename):
    bag = rosbag.Bag(filename)

    print("Creating " + outfilename)
    f = open(outfilename, 'w')

    f.write('''<?xml version="1.0" encoding="UTF-8"?>
<gpx
  version="1.0"
  creator="ACS"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xmlns="http://www.topografix.com/GPX/1/0"
  xsi:schemaLocation="http://www.topografix.com/GPX/1/0 http://www.topografix.com/GPX/1/0/gpx.xsd">
<trk>
<trkseg>
''')

    prev_time = 0
    prev_lat = -1000.0  #not using 0 because that's a valid lat
    prev_lon = -1000.0  #not using 0 because that's a valid lon

    for topic, msg, t in bag.read_messages(topics=['/autopilot/acs_pose']):
        tim = float(str(t)) / 1e9

        if opts.mintime != 0 and tim < opts.mintime:
            continue
        if opts.maxtime != 0 and tim > opts.maxtime:
            continue

        lat = msg.pose.pose.position.lat
        lon = msg.pose.pose.position.lon
        alt = msg.pose.pose.position.alt

        t = time.localtime(tim)
    
        #TODO: repair heading
        eulers = tf.transformations.euler_from_quaternion((
                msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        hdg = math.degrees(eulers[2])

        #TODO: repair velocity
        if prev_lat != -1000.0:
            delta_time = tim - prev_time
            dis = gps_utils.gps_distance(prev_lat, prev_lon, lat, lon)
            v = dis / delta_time
        else:
            v = 0.0

        """
        HEADING and VELOCITY are not yet correct, so not
        showing course and speed in output

        f.write('''<trkpt lat="%0.07f" lon="%0.07f">
  <ele>%0.02f</ele>
  <time>%s</time>
  <course>%0.02f</course>
  <speed>%0.02f</speed>
  <fix>3d</fix>
</trkpt>
''' % (lat, lon, alt,
       time.strftime("%Y-%m-%dT%H:%M:%SZ", t),
       hdg, v))
    """
    
        f.write('''<trkpt lat="%0.07f" lon="%0.07f">
  <ele>%0.02f</ele>
  <time>%s</time>
  <fix>3d</fix>
</trkpt>
''' % (lat, lon, alt,
       time.strftime("%Y-%m-%dT%H:%M:%SZ", t) ))

        prev_time = tim
        prev_lat = lat
        prev_lon = lon

    f.write('''</trkseg>
</trk>
</gpx>
''')

    f.close()
    bag.close()

if opts.dir is None and len(args) < 2:
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
                    out_file_path = os.path.join(current_dir,file_name + ".gpx")
                    extractgpx(fullpath, out_file_path)

    else:
        extractgpx(args[0], args[1])

