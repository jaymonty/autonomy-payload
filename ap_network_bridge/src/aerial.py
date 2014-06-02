#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-Network Bridge (UDP send/recv of aircraft state and commands)
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
import ctypes
import netifaces
from optparse import OptionParser
import socket
import struct
import sys

# General ROS imports
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# Import ROS messages specific to this bridge
from ap_network_bridge import msg as netmsg
from ap_autopilot_bridge import msg as apmsg

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
aircraft_bcast = 0xffff

# UDP socket and info
udp_sock = None
udp_sock_local_ip = None
udp_sock_bcast_ip = None
udp_sock_gcs_ip = None
udp_sock_arbiter_ip = None
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
# UDP messaging functions

def sock_init(device, port, gcs_ip=None, arbiter_ip=None):
    global udp_sock, udp_sock_local_ip, udp_sock_port
    global udp_sock_gcs_ip, udp_sock_arbiter_ip, udp_sock_bcast_ip
    
    udp_sock_port = port
    udp_sock_gcs_ip = gcs_ip
    udp_sock_arbiter_ip = arbiter_ip
    
    if device == 'lo':
        # Assumes 'lo' is only used for single-UAV testing with SITL,
        # broadcast solution doesn't really work with multiple UAS
        udp_sock_local_ip = '127.0.0.1'
        udp_sock_bcast_ip = '127.0.1.1'
    else:
        try:
            udp_sock_local_ip = netifaces.ifaddresses(device)[2][0]['addr']
            udp_sock_bcast_ip = netifaces.ifaddresses(device)[2][0]['broadcast']
        except Exception:
            log_warn("Could not get address info for device %s!"%device)
            return False
    
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    udp_sock.bind((udp_sock_local_ip, udp_sock_port))
    return True

'''
Packet header format (all fields in network byte order):
 - (8b)  Message type
 - (8b)  RESERVED
 - (16b) Aircraft ID
 - (32b) Seconds since Unix epoch
 - (32b) Nanoseconds since last second
'''
HEADER_FORMAT = ">BBHLL"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

PACKET_TYPES = {  # Aircraft-originated messages
                  0x00 : { 'fmt' : '>' },	# Flight system status
                  0x01 : { 'fmt' : '>lllllll',	# Aircraft Pose
                           'cb'  : None },
                  # Ground-originated messages
                  0x80 : { 'fmt' : '>',		# 
                           'cb'  : None },
                  0x81 : { 'fmt' : '>',		# 
                           'cb'  : None },
                  0x82 : { 'fmt' : '>',		# 
                           'cb'  : None },
                  0x83 : { 'fmt' : '>',		# 
                           'cb'  : None },
                  0x84 : { 'fmt' : '>',		# 
                           'cb'  : None },
                  0x85 : { 'fmt' : '>',		# 
                           'cb'  : None },
                  0xFF : { 'fmt' : '>',		# 
                           'cb'  : None } }

# Compute size of payload, and cache it for future lookup
def payload_size(msg_type):
    if 'sz' not in PACKET_TYPES[msg_type]:
        PACKET_TYPES[msg_type]['sz'] = \
            struct.calcsize(PACKET_TYPES[msg_type]['fmt'])
    return PACKET_TYPES[msg_type]['sz']

def send(ip, msg_type, msg_id, msg_secs, msg_nsecs, pload_fields):
    global udp_sock, udp_sock_port
    
    if msg_type not in PACKET_TYPES:
        return False
    
    try:
        pload_size = payload_size(msg_type)
        msg = struct.pack(HEADER_FORMAT,
                          msg_type, 0x00, msg_id, msg_secs, msg_nsecs)
        if pload_size and pload_fields:
            msg = msg + struct.pack(PACKET_TYPES[msg_type]['fmt'],
                                    *pload_fields)
        
        return udp_sock.sendto(msg, (ip, udp_sock_port))
    except Exception as ex:
        log_warn(ex.args)
        return False

def recv():
    global udp_sock, udp_sock_local_ip
    global aircraft_id, aircraft_bcast
    
    # Attempt to receive a packet
    message = None
    try:
        message, (ip, port) = udp_sock.recvfrom(1024, socket.MSG_DONTWAIT)
        if ip == udp_sock_local_ip:
            return False
    except Exception as ex:
        return None
    if not message:
        return None
    
    msg_type, _unused, msg_id, msg_secs, msg_nsecs = \
        struct.unpack_from(HEADER_FORMAT, message, 0)
    
    # Is it a recognized message type?
    if msg_type not in PACKET_TYPES:
        return False
    # If ground-originated, is it meant for us?
    if (msg_type >= 0x80) and (msg_id != aircraft_id) \
                          and (msg_id != aircraft_bcast):
        return False
    
    # Create tuples of headers, and fields if any
    headers = (msg_type, msg_id, msg_secs, msg_nsecs)
    pload_size = payload_size(msg_type)
    fields = None
    if pload_size and (len(message) == (HEADER_SIZE + pload_size)):
        fields = struct.unpack_from(PACKET_TYPES[msg_type]['fmt'],
                                    message,
                                    HEADER_SIZE)
    
    # If a callback exists, use it; otherwise, return tuples
    if 'cb' in PACKET_TYPES[msg_type] and PACKET_TYPES[msg_type]['cb']:
        return PACKET_TYPES[msg_type]['cb'](headers, fields)
    return (headers, fields)

#-----------------------------------------------------------------------
# Subscribers (ROS -> Network)

def sub_status(msg):
    global aircraft_id
    global udp_sock_gcs_ip
    
    True

def sub_pose(msg):
    global aircraft_id
    global udp_sock_bcast_ip
    
    send(udp_sock_bcast_ip,
         0x01,
         aircraft_id, 
         msg.header.stamp.secs,
         msg.header.stamp.nsecs,
         (int(msg.pose.pose.position.x * 1e07), 
          int(msg.pose.pose.position.y * 1e07), 
          int(msg.pose.pose.position.z * 1e03), 
          int(msg.pose.pose.orientation.x * 1e09),
          int(msg.pose.pose.orientation.y * 1e09),
          int(msg.pose.pose.orientation.z * 1e09),
          int(msg.pose.pose.orientation.w * 1e09)))

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("aerial.py [options]")
    parser.add_option("--id", dest="acid", type='int',
                      help="Aircraft ID", default=1)
    parser.add_option("--device", dest="device", 
                      help="Network device to use", default="wlan0")
    parser.add_option("--port", dest="port", type='int',
                      help="UDP port", default=5554)
    (opts, args) = parser.parse_args()
    
    # Initialize ROS
    rospy.init_node('net_aerial')
    aircraft_id = opts.acid
    
    # Initialize socket
    if not sock_init(opts.device, opts.port):
        sys.exit(-1)
    
    # Set up subscribers (ROS -> network)
    rospy.Subscriber("%s/send_flight_status"%ROS_BASENAME, 
                     apmsg.Status, sub_status)
    rospy.Subscriber("%s/send_pose"%ROS_BASENAME, 
                     PoseWithCovarianceStamped, sub_pose)
    
    # Set up publishers (network -> ROS)
    pub_pose = rospy.Publisher("%s/recv_pose"%ROS_BASENAME, 
                               netmsg.NetPoseStamped)
    
    # Loop , checking for incoming datagrams and sleeping
    # NOTE: If too many network messages come in, this loop
    #  might not get around to sleeping (which means subscriber
    #  callbacks won't run)
    r = rospy.Rate(10)
    print "\nStarting aerial comms loop...\n"
    while not rospy.is_shutdown():
        message = recv()
        if message == False:  # Got packet, not for us or not valid
            continue
        if message == None:   # No packet to get, sleep a bit
            r.sleep()
            continue
        
        ((msg_type, msg_id, msg_secs, msg_nsecs), fields) = message
        
        if msg_type == 0x01:
            msg = netmsg.NetPoseStamped()
            msg.sender_id = msg_id 
            msg.pose.header.stamp.secs = msg_secs
            msg.pose.header.stamp.nsecs = msg_nsecs
            msg.pose.header.seq = 0
            (lat, lon, alt, q_x, q_y, q_z, q_w) = fields
            msg.pose.pose.position.x = lat / 1e07 
            msg.pose.pose.position.y = lon / 1e07
            msg.pose.pose.position.z = alt /1e03
            msg.pose.pose.orientation.x = q_x / 1e09
            msg.pose.pose.orientation.y = q_y / 1e09
            msg.pose.pose.orientation.z = q_z / 1e09
            msg.pose.pose.orientation.w = q_w / 1e09
            pub_pose.publish(msg)
            
        elif msg_type == 0x80:
            True
            
        elif msg_type == 0x81:
            True
            
        elif msg_type == 0x82:
            True
            
        elif msg_type == 0x83:
            True
            
        elif msg_type == 0x84:
            True
            
        elif msg_type == 0x85:
            True
            
        elif msg_type == 0xFF:
            True
        

