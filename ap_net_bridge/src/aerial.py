#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-Network Bridge (UDP broadcast send/recv of GPS data)
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
#import os
#import sys
#import signal
import struct
import socket
from optparse import OptionParser

# General ROS imports
#import roslib; roslib.load_manifest('ap_mavlink_bridge')
import rospy
#from std_msgs.msg import String, Header
#from std_srvs.srv import Empty
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

# Import ROS messages specific to this bridge
from ap_network_bridge import msg as netmsg

#-----------------------------------------------------------------------
# Parameters

# Base name for node topics and services
ROS_BASENAME = 'aerial'

# Control printing of messages to stdout
DBUG_PRINT = False
WARN_PRINT = False

#-----------------------------------------------------------------------
# Ugly global variables

# Aircraft ID
aircraft_id = None

# UDP socket and info
udp_sock = None
udp_sock_ip = None
udp_sock_port = None

#-----------------------------------------------------------------------
# logging functions

def log_dbug(msg):
    rospy.logdebug(msg)
    if DBUG_PRINT:
        print "..DEBUG.. %s" % msg

def log_warn(msg):
    rospy.logwarn(msg)
    if WARN_PRINT:
        print "**WARN** %s" % msg

#-----------------------------------------------------------------------
# UDP socket functions

def sock_init(remote_ip, port):
    global udp_sock, udp_sock_ip, udp_sock_port
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind(('', port))
    udp_sock_ip = remote_ip
    udp_sock_port = port

def sock_send(data):
    global udp_sock, udp_sock_ip, udp_sock_port
    udp_sock.sendto(data, (udp_sock_ip, udp_sock_port))

def sock_recv(max_size=1024):
    data = None
    try:
        data, addr = udp_sock.recvfrom(max_size, socket.MSG_DONTWAIT)
    except Exception:
        True
    return data

#-----------------------------------------------------------------------
# Byte packing and unpacking

PACKET_FORMAT = ">HHLLllllll"
'''
Packet fields (in network byte order):
 - (16b) Aircraft ID (1..65535)
 - (16b) Message sequence number (1..65535)
 - (32b) Seconds since epoch
 - (32b) Nanoseconds since last second
 - (32b) Latitude (deg) * 1e07 (truncated)
 - (32b) Longitude (deg) * 1e07 (truncated)
 - (32b) Altitude (m) * 1e03 (truncated)
 - (32b) Roll (rads) * 1e07 (truncated)
 - (32b) Pitch (rads) * 1e07 (truncated)
 - (32b) Yaw (rads) * 1e07 (truncated)
'''

def gps_pack(acid, seq, stamp, lat, lon, alt, rol, pit, yaw):
    sec = int(stamp.secs)
    nsec = int(stamp.nsecs)
    lat = int(lat * 1e07)
    lon = int(lon * 1e07)
    alt = int(alt * 1e03)
    rol = int(rol * 1e07)
    pit = int(pit * 1e07)
    yaw = int(yaw * 1e07)
    return struct.pack(PACKET_FORMAT, acid, seq, sec, nsec, 
                       lat, lon, alt, rol, pit, yaw)

def gps_unpack(data):
    acid, seq, sec, nsec, lat, lon, alt, rol, pit, yaw \
        = struct.unpack(PACKET_FORMAT, data)
    stamp = rospy.Time()
    stamp.secs = sec
    stamp.nsecs = nsec
    lat = float(lat) / 1e07
    lon = float(lon) / 1e07
    alt = float(alt) / 1e03
    rol = float(rol) / 1e07
    pit = float(pit) / 1e07
    yaw = float(yaw) / 1e07
    return acid, seq, stamp, lat, lon, alt, rol, pit, yaw

#-----------------------------------------------------------------------
# Subscribers (ROS -> Network)

def sub_mavlink_gpsraw(msg):
    global aircraft_id
    
    # Package contents into a datagram
    dgram = gps_pack(aircraft_id, msg.header.seq, msg.header.stamp,
                     msg.latitude, msg.longitude, msg.altitude)
    
    # Send out via network
    sock_send(dgram)

#-----------------------------------------------------------------------
# Start-up

def on_ros_shutdown():
    True

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("gps_bcast_rxtx.py [options]")
    parser.add_option("--id", dest="acid", type='int',
                      help="Aircraft ID", default=1)
    parser.add_option("--ip", dest="ip", 
                      help="IP address to send to", default="192.168.1.1")
    parser.add_option("--port", dest="port", type='int',
                      help="UDP port", default=5554)
    parser.add_option("--sub", dest="sub",
                      help="ROS topic to subscribe to", 
                      default="mavlink/gps_raw")
    parser.add_option("--pub", dest="pub",
                      help="ROS topic to publish to", 
                      default="net/gps_bcast")
    (opts, args) = parser.parse_args()
    
    # Initialize ROS and socket
    rospy.init_node('ap_network_bridge_gpsbc')
    rospy.on_shutdown(on_ros_shutdown)
    sock_init(opts.ip, opts.port)
    aircraft_id = opts.acid
    
    # Set up subscriber (ROS -> network)
    rospy.Subscriber(opts.sub, NavSatFix, sub_mavlink_gpsraw)
    
    # Set up publisher (network -> ROS)
    pub = rospy.Publisher(opts.pub, netmsg.shared_lla)
    
    # Loop , checking for incoming datagrams and sleeping
    r = rospy.Rate(10)
    print "\nStarting loop...\n"
    while not rospy.is_shutdown():
        # Check for datagrams
        while (True):
            dgram = sock_recv()
            if not dgram:
                break
            
            # Convert into ROS message and publish
            acid, seq, stamp, lat, lon, alt, rol, pit, yaw \
                = gps_unpack(dgram)
            msg = netmsg.shared_lla(acid, seq, stamp, 
                                    lat, lon, alt, rol, pit, yaw)
            pub.publish(msg)
        
        # No more messages, so sleep briefly
        r.sleep()



