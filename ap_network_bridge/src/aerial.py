#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-Network Bridge (UDP send/recv of aircraft state and commands)
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
import struct
import socket
import netifaces
from optparse import OptionParser

# General ROS imports
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

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
# UDP socket functions

def sock_init(device, port, gcs_ip=None, arbiter_ip=None):
    global udp_sock, udp_sock_local_ip, udp_sock_bcast_ip, udp_sock_port
    
    udp_sock_port = port
    udp_sock_gcs_ip = gcs_ip
    udp_sock_arbiter_ip = arbiter_ip
    
    if device == 'lo':
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

def sock_send_gcs(data):
    global udp_sock, udp_sock_bcast_ip, udp_sock_port
    udp_sock.sendto(data, (udp_sock_gcs_ip, udp_sock_port))

def sock_send_arbiter(data):
    global udp_sock, udp_sock_bcast_ip, udp_sock_port
    udp_sock.sendto(data, (udp_sock_arbiter_ip, udp_sock_port))

def sock_send_bcast(data):
    global udp_sock, udp_sock_bcast_ip, udp_sock_port
    udp_sock.sendto(data, (udp_sock_bcast_ip, udp_sock_port))

def sock_recv(max_size=1024):
    data = None
    try:
        data, (ip, port) = udp_sock.recvfrom(max_size, socket.MSG_DONTWAIT)
        if ip == udp_sock_local_ip:
            return False
    except Exception:
        return None
    return data

#-----------------------------------------------------------------------
# Byte packing and unpacking

PACKET_FORMAT = ">HHLLLlllllll"
'''
Packet fields (in network byte order):
 - (16b) Aircraft ID (1..65535)
 - (16b) RESERVED
 - (32b) Message sequence number (1..4Bil)
 - (32b) Seconds since epoch
 - (32b) Nanoseconds since last second
 - (32b) Latitude (deg) * 1e07 (truncated)
 - (32b) Longitude (deg) * 1e07 (truncated)
 - (32b) Altitude (m) * 1e03 (truncated)
 - (32b) Quat X (?) * 1e09 (truncated)
 - (32b) Quat Y (?) * 1e09 (truncated)
 - (32b) Quat Z (?) * 1e09 (truncated)
 - (32b) Quat W (?) * 1e09 (truncated)
'''

def pose_pack(acid, seq, secs, nsecs, lat, lon, alt, q_x, q_y, q_z, q_w):
    return struct.pack(PACKET_FORMAT, 
                       acid, 
                       0x0000,
                       seq,
                       secs,
                       nsecs,
                       int(lat * 1e07),
                       int(lon * 1e07),
                       int(alt * 1e03),
                       int(q_x * 1e09),
                       int(q_y * 1e09),
                       int(q_z * 1e09),
                       int(q_w * 1e09))

def pose_unpack(data):
    acid, unused1, seq, secs, nsecs, lat, lon, alt, q_x, q_y, q_z, q_w \
        = struct.unpack(PACKET_FORMAT, data)
    lat = float(lat) / 1e07
    lon = float(lon) / 1e07
    alt = float(alt) / 1e03
    q_x = float(q_x) / 1e09
    q_y = float(q_y) / 1e09
    q_z = float(q_z) / 1e09
    q_w = float(q_w) / 1e09
    return acid, seq, secs, nsecs, lat, lon, alt, q_x, q_y, q_z, q_w

#-----------------------------------------------------------------------
# Subscribers (ROS -> Network)

def sub_pose(msg):
    global aircraft_id
    
    # Package contents into a datagram
    dgram = pose_pack(aircraft_id, 
                      msg.header.seq, 
                      msg.header.stamp.secs,
                      msg.header.stamp.nsecs,
                      msg.pose.pose.position.x, 
                      msg.pose.pose.position.y, 
                      msg.pose.pose.position.z, 
                      msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w)
    # Send out via network
    sock_send_bcast(dgram)

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
    parser.add_option("--sub", dest="sub",
                      help="ROS topic to subscribe to", 
                      default="estimator/odom_combined")
    parser.add_option("--pub", dest="pub",
                      help="ROS topic to publish to", 
                      default="network/pose")
    (opts, args) = parser.parse_args()
    
    # Initialize ROS
    rospy.init_node('net_aerial')
    aircraft_id = opts.acid
    
    # Initialize socket
    if not sock_init(opts.device, opts.port):
        sys.exit(-1)
    
    # Set up subscriber (ROS -> network)
    rospy.Subscriber(opts.sub, PoseWithCovarianceStamped, sub_pose)
    
    # Set up publisher (network -> ROS)
    pub = rospy.Publisher(opts.pub, netmsg.NetPoseStamped)
    
    # Loop , checking for incoming datagrams and sleeping
    r = rospy.Rate(10)
    print "\nStarting aerial comms loop...\n"
    while not rospy.is_shutdown():
        # Check for datagrams
        while (True):
            dgram = sock_recv()
            if dgram == False:  # Got packet, not for us
                continue
            if dgram == None:   # No packet to get
                break
            
            # Convert into ROS message and publish
            acid, seq, secs, nsecs, lat, lon, alt, q_x, q_y, q_z, q_w \
                = pose_unpack(dgram)
            msg = netmsg.NetPoseStamped()
            msg.sender_id = acid 
            msg.pose.header.stamp.secs = secs
            msg.pose.header.stamp.nsecs = nsecs
            msg.pose.header.seq = seq 
            msg.pose.pose.position.x = lat 
            msg.pose.pose.position.y = lon 
            msg.pose.pose.position.z = alt 
            msg.pose.pose.orientation.x = q_x
            msg.pose.pose.orientation.y = q_y
            msg.pose.pose.orientation.z = q_z
            msg.pose.pose.orientation.w = q_w
            pub.publish(msg)
        
        # No more messages, so sleep briefly
        r.sleep()



