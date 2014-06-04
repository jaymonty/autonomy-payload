#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-Network Bridge (UDP send/recv of aircraft state and commands)
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
from optparse import OptionParser
import sys

# Load in ACS libraries
import os, inspect
LIB_ACS_PATH = os.path.realpath(
                   os.path.abspath(
                       os.path.join(
                           os.path.split(inspect.getfile(inspect.currentframe()))[0],
                           "../../lib/acs")))
sys.path.insert(0, LIB_ACS_PATH)
import acs_messages
import acs_socket

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

#-----------------------------------------------------------------------
# Subscribers (ROS -> Network)

def sub_flight_status(msg):
    message = acs_messages.FlightStatus()
    message.msg_dst = acs_socket.ID_BCAST_ALL  # TODO: Send to Ground only?
    message.msg_secs = msg.header.stamp.secs
    message.msg_nsecs = msg.header.stamp.nsecs
    message.mode = msg.mode
    message.armed = msg.armed
    message.ok_ahrs = msg.ahrs_ok
    message.ok_as = msg.as_ok
    message.ok_gps = msg.gps_ok
    message.ok_ins = msg.ins_ok
    message.ok_mag = msg.mag_ok
    message.ok_pwr = msg.pwr_ok
    message.gps_sats = msg.gps_sats
    message.batt_rem = msg.pwr_batt_rem
    message.batt_vcc = msg.pwr_batt_vcc
    message.batt_cur = msg.pwr_batt_cur
    message.airspeed = msg.as_read
    message.alt_rel = msg.alt_rel
    message.gps_hdop = msg.gps_eph
    acs_socket.send(message)

def sub_pose(msg):
    message = acs_messages.Pose()
    message.msg_dst = acs_socket.ID_BCAST_ALL
    message.msg_secs = msg.header.stamp.secs
    message.msg_nsecs = msg.header.stamp.nsecs
    message.lat = msg.pose.pose.position.x
    message.lon = msg.pose.pose.position.y
    message.alt = msg.pose.pose.position.z
    message.q_x = msg.pose.pose.orientation.x
    message.q_y = msg.pose.pose.orientation.y
    message.q_z = msg.pose.pose.orientation.z
    message.q_w = msg.pose.pose.orientation.w
    acs_socket.send(message)

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
    
    # Initialize socket
    # TODO: Create dictionary of IDs->IPs in place of 'None'
    if not acs_socket.init(opts.device, opts.port, opts.acid, None):
        rospy.logfatal("Could not initialize network socket")
        sys.exit(-1)
    
    # Set up subscribers (ROS -> network)
    rospy.Subscriber("%s/send_flight_status"%ROS_BASENAME, 
                     apmsg.Status, sub_flight_status)
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
        message = acs_socket.recv()
        if message == False:  # Got packet, not for us or not valid
            continue
        if message == None:   # No packet to get, sleep a bit
            r.sleep()
            continue
        
        if isinstance(message, acs_messages.Pose):
            msg = netmsg.NetPoseStamped()
            msg.sender_id = message.msg_src
            msg.pose.header.stamp.secs = message.msg_secs
            msg.pose.header.stamp.nsecs = message.msg_nsecs
            msg.pose.header.seq = 0
            msg.pose.pose.position.x = message.lat
            msg.pose.pose.position.y = message.lon
            msg.pose.pose.position.z = message.alt
            msg.pose.pose.orientation.x = message.q_x
            msg.pose.pose.orientation.y = message.q_y
            msg.pose.pose.orientation.z = message.q_z
            msg.pose.pose.orientation.w = message.q_w
            pub_pose.publish(msg)
            
        elif False:  # Add other message types here
            True
        

