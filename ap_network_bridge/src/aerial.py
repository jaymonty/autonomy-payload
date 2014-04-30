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
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

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
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    udp_sock_ip = remote_ip
    udp_sock_port = port

def sock_send(data):
    global udp_sock, udp_sock_ip, udp_sock_port
    udp_sock.sendto(data, (udp_sock_ip, udp_sock_port))

def sock_recv(max_size=1024):
    data = None
    try:
        data, addr = udp_sock.recvfrom(max_size, socket.MSG_DONTWAIT)
        # TODO check addr to see if it's me (and then loop?)
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

def sub_pose(msg):
    global aircraft_id
    
    # Package contents into a datagram
    rol, pit, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                           msg.pose.pose.orientation.y,
                                           msg.pose.pose.orientation.z,
                                           msg.pose.pose.orientation.w],
                                           'sxyz')
    dgram = gps_pack(aircraft_id, msg.header.seq, msg.header.stamp,
                     msg.pose.pose.position.x, 
                     msg.pose.pose.position.y, 
                     msg.pose.pose.position.z, 
                     rol, pit, yaw)
    log_dbug("(aerial) X: %f Y: %f Z: %f R: %f P: %f Y: %f" % \
             (msg.pose.pose.position.x, msg.pose.pose.position.y,
              msg.pose.pose.position.z, rol, pit, yaw))
    # Send out via network
    sock_send(dgram)

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("aerial.py [options]")
    parser.add_option("--id", dest="acid", type='int',
                      help="Aircraft ID", default=1)
    parser.add_option("--ip", dest="ip", 
                      help="IP address to send to", default="192.168.2.1")
    parser.add_option("--port", dest="port", type='int',
                      help="UDP port", default=5554)
    parser.add_option("--sub", dest="sub",
                      help="ROS topic to subscribe to", 
                      default="estimator/odom")
    parser.add_option("--pub", dest="pub",
                      help="ROS topic to publish to", 
                      default="net/pose")
    (opts, args) = parser.parse_args()
    
    # Initialize ROS and socket
    rospy.init_node('net_aerial')
    sock_init(opts.ip, opts.port)
    aircraft_id = opts.acid
    
    # Set up subscriber (ROS -> network)
    rospy.Subscriber(opts.sub, PoseWithCovarianceStamped, sub_pose)
    
    # Set up publisher (network -> ROS)
    pub = rospy.Publisher(opts.pub, netmsg.shared_lla)
    
    # Loop , checking for incoming datagrams and sleeping
    r = rospy.Rate(10)
    print "\nStarting loop...\n"
    while not rospy.is_shutdown():
        # Check for datagrams
        while (True):
            break # TODO fix self-listening behavior in sock_recv()

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



