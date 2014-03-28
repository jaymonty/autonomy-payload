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
from ap_mavlink_bridge import msg as mavmsg

#-----------------------------------------------------------------------
# Parameters

# Base name for node topics and services
ROS_BASENAME = 'mavlink'

# Control printing of messages to stdout
DBUG_PRINT = False
WARN_PRINT = False

#-----------------------------------------------------------------------
# Ugly global variables

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
    udp_sock.bind((socket.INADDR_ANY, port))
    udp_sock_ip = remote_ip
    udp_sock_port = port

def sock_send(data):
    global udp_sock, udp_sock_ip, udp_sock_port
    udp_sock.sendto(data, (udp_sock_ip, udp_sock_port))

def sock_recv(max_size=1024):
    data, addr = udp_sock.recvfrom(max_size, socket.MSG_DONTWAIT)
    return data

#-----------------------------------------------------------------------
# Subscribers (ROS -> Network)

def sub_mavlink_gpsraw(data):
    # Package contents into a datagram
    dgram = None
    
    # Send out via network
    sock_send(dgram)

#-----------------------------------------------------------------------
# Start-up

def on_ros_shutdown():
    True

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("gps_bcast_rxtx.py [options]")
    parser.add_option("--device", dest="device", 
                      help="serial device", default="auto-detect")
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="serial baud rate", default=57600)
    parser.add_option("--mavlinkdir", dest="mavlink_dir", 
                      help="path to mavlink folder", default=None)
    (opts, args) = parser.parse_args()
    
    # Initialize ROS and socket
    rospy.init_node('ap_network_bridge_gpsbc')
    rospy.on_shutdown(on_ros_shutdown)
    
    # Set up subscriber (ROS -> network)
    rospy.Subscriber("mavlink/gps_raw", 
                     NavSatFix, 
                     sub_mavlink_gpsraw)
    
    # Set up publisher (network -> ROS)
    
    
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
            
        
        # No more messages, so sleep briefly
        r.sleep()



