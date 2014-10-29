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
import os
import sys

# Load in ACS libraries
import ap_lib.acs_messages as acs_messages
from ap_lib.acs_socket import Socket

# General ROS imports
import rospy
import std_msgs

# Import ROS messages specific to this bridge
from ap_msgs import msg as ap_msgs
from ap_srvs import srv as ap_srvs
from autopilot_bridge import msg as apmsg
from autopilot_bridge import srv as apsrv

#-----------------------------------------------------------------------
# Parameters

# Base name for node topics and services
ROS_BASENAME = 'network'

#-----------------------------------------------------------------------
# Ugly global data

acs_sock = None     # socket
aircraft_id = 0     # aircraft numeric ID
aircraft_name = ''  # aircraft friendly name

#-----------------------------------------------------------------------
# Subscribers (ROS -> Network)

def sub_flight_status(msg):
    global acs_sock, aircraft_name
    message = acs_messages.FlightStatus()
    message.msg_dst = Socket.ID_BCAST_ALL  # TODO: Send to ground
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
    message.mis_cur = msg.mis_cur

    # Also bring in flight-ready flag
    message.ready = rospy.has_param("flight_ready") and \
                    bool(rospy.get_param("flight_ready"))

    # Add friendly name
    message.name = aircraft_name

    acs_sock.send(message)

def sub_pose(msg):
    global acs_sock
    message = acs_messages.Pose()
    message.msg_dst = Socket.ID_BCAST_ALL
    message.msg_secs = msg.header.stamp.secs
    message.msg_nsecs = msg.header.stamp.nsecs
    message.lat = msg.pose.pose.position.lat
    message.lon = msg.pose.pose.position.lon
    message.alt = msg.pose.pose.position.alt  # NOTE: send MSL
    message.q_x = msg.pose.pose.orientation.x
    message.q_y = msg.pose.pose.orientation.y
    message.q_z = msg.pose.pose.orientation.z
    message.q_w = msg.pose.pose.orientation.w
    message.vlx = msg.twist.twist.linear.x
    message.vly = msg.twist.twist.linear.y
    message.vlz = msg.twist.twist.linear.z
    message.vax = msg.twist.twist.angular.x
    message.vay = msg.twist.twist.angular.y
    message.vaz = msg.twist.twist.angular.z
    acs_sock.send(message)

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("aerial.py [options]")
    parser.add_option("--id", dest="acid", type='int',
                      help="Aircraft ID (default to ROS param aircraft_id)", default=0)
    parser.add_option("--name", dest="acname",
                      help="Aircraft Name (default to ROS param aircraft_name)", default='')
    parser.add_option("--device", dest="device", 
                      help="Network device to use (default wlan0)", default="wlan0")
    parser.add_option("--port", dest="port", type='int',
                      help="UDP port (default 5554)", default=5554)
    (opts, args) = parser.parse_args()
    
    # Initialize ROS
    rospy.init_node(ROS_BASENAME)
    
    # Make sure ID and Name are set
    if opts.acid > 0:
        aircraft_id = opts.acid
    elif rospy.has_param("aircraft_id"):
        aircraft_id = rospy.get_param("aircraft_id")
    else:
        rospy.logfatal("Aircraft ID not specified and no aircraft_id param")
        sys.exit(-1)
    if opts.acname != '':
        aircraft_name = opts.acname
    elif rospy.has_param("aircraft_name"):
        aircraft_name = rospy.get_param("aircraft_name")
    else:
        rospy.logfatal("Aircraft name not specified and no aircraft_name param")
        sys.exit(-1)

    # Initialize socket
    # TODO: Create dictionary of IDs->IPs 
    try:
        loc_ip = None
        rem_ip = None
        if opts.device == 'lo':
            loc_ip = '127.0.0.1'
            rem_ip = '127.0.1.1'
        acs_sock = Socket(aircraft_id, opts.port, opts.device, 
                          loc_ip, rem_ip, None)
    except Exception:
        rospy.logfatal("Could not initialize network socket")
        sys.exit(-1)
    
    # Set up subscribers (ROS -> network)
    rospy.Subscriber("%s/send_flight_status"%ROS_BASENAME, 
                     apmsg.Status, sub_flight_status)
    rospy.Subscriber("%s/send_pose"%ROS_BASENAME, 
                     apmsg.Geodometry, sub_pose)
    
    # Set up publishers (network -> ROS)
    pub_pose = rospy.Publisher("%s/recv_pose"%ROS_BASENAME, 
                               ap_msgs.SwarmVehicleState)
    pub_heart_ground = rospy.Publisher("%s/recv_heart_ground"%ROS_BASENAME,
                                apmsg.Heartbeat)
    pub_arm = rospy.Publisher("%s/recv_arm"%ROS_BASENAME, 
                              std_msgs.msg.Bool)
    pub_mode = rospy.Publisher("%s/recv_mode"%ROS_BASENAME, 
                               std_msgs.msg.UInt8)
    pub_land = rospy.Publisher("%s/recv_land"%ROS_BASENAME, 
                               std_msgs.msg.Empty)
    pub_land_abort = rospy.Publisher("%s/recv_land_abort"%ROS_BASENAME, 
                                     std_msgs.msg.UInt16)
    pub_guided_goto = rospy.Publisher("%s/recv_guided_goto"%ROS_BASENAME, 
                                      apmsg.LLA)
    pub_waypoint_goto = rospy.Publisher("%s/recv_waypoint_goto"%ROS_BASENAME, 
                                        std_msgs.msg.UInt16)
    
    # Loop , checking for incoming datagrams and sleeping
    # NOTE: If too many network messages come in, this loop
    #  might not get around to sleeping (which means subscriber
    #  callbacks won't run)
    r = rospy.Rate(10)
    print "\nStarting aerial comms loop...\n"
    while not rospy.is_shutdown():
        message = acs_sock.recv()
        if message == False:  # Got packet, not for us or not valid
            continue
        if message == None:   # No packet to get, sleep a bit
            r.sleep()
            continue
        
        # Aircraft -> Aircraft messages
        
        if isinstance(message, acs_messages.Pose):
            try:
                msg = ap_msgs.SwarmVehicleState()
                msg.vehicle_id = message.msg_src
                msg.subswarm_id = message.msg_sub
                msg.state.header.stamp.secs = message.msg_secs
                msg.state.header.stamp.nsecs = message.msg_nsecs
                msg.state.header.seq = 0
                msg.state.pose.pose.position.lat = message.lat
                msg.state.pose.pose.position.lon = message.lon
                msg.state.pose.pose.position.alt = message.alt
                msg.state.pose.pose.position.rel_alt = 0
                msg.state.pose.pose.position.using_alt = True
                msg.state.pose.pose.position.using_rel_alt = False
                msg.state.pose.pose.orientation.x = message.q_x
                msg.state.pose.pose.orientation.y = message.q_y
                msg.state.pose.pose.orientation.z = message.q_z
                msg.state.pose.pose.orientation.w = message.q_w
                # msg.state.pose.covariance is not used
                msg.state.twist.twist.linear.x = message.vlx
                msg.state.twist.twist.linear.y = message.vly
                msg.state.twist.twist.linear.z = message.vlz
                msg.state.twist.twist.angular.x = message.vax
                msg.state.twist.twist.angular.y = message.vay
                msg.state.twist.twist.angular.z = message.vaz
                # msg.state.twist.covariance is not used
                pub_pose.publish(msg)
            except:
                rospy.logwarn("Error processing received Pose")
            
        # Ground -> Aircraft messages
        
        elif isinstance(message, acs_messages.Heartbeat):
            try:
                msg = apmsg.Heartbeat()
                msg.counter = message.counter
                pub_heart_ground.publish(msg)
                #rospy.loginfo("Ground-to-air: Heartbeat")
            except:
                rospy.logwarn("Error processing command: Heartbeat")
            
        elif isinstance(message, acs_messages.Arm):
            try:
                msg = std_msgs.msg.Bool()
                msg.data = message.enable
                pub_arm.publish(msg)
                rospy.loginfo("Ground-to-air: Arm")
            except:
                rospy.logwarn("Error processing command: Arm")
            
        elif isinstance(message, acs_messages.Mode):
            try:
                msg = std_msgs.msg.UInt8()
                msg.data = message.mode
                pub_mode.publish(msg)
                rospy.loginfo("Ground-to-air: Mode")
            except:
                rospy.logwarn("Error processing command: Mode")
            
        elif isinstance(message, acs_messages.Land):
            try:
                msg = std_msgs.msg.Empty()
                pub_land.publish(msg)
                rospy.loginfo("Ground-to-air: Land")
            except:
                rospy.logwarn("Error processing command: Land")
            
        elif isinstance(message, acs_messages.LandAbort):
            try:
                msg = std_msgs.msg.UInt16()
                msg.data = message.alt
                pub_land_abort.publish(msg)
                rospy.loginfo("Ground-to-air: LandAbort")
            except:
                rospy.logwarn("Error processing command: LandAbort")
            
        elif isinstance(message, acs_messages.GuidedGoto):
            try:
                msg = apmsg.LLA()
                msg.lat = message.lat
                msg.lon = message.lon
                msg.alt = message.alt
                pub_guided_goto.publish(msg)
                rospy.loginfo("Ground-to-air: GuidedGoto")
            except:
                rospy.logwarn("Error processing command: GuidedGoto")
            
        elif isinstance(message, acs_messages.WaypointGoto):
            try:
                msg = std_msgs.msg.UInt16()
                msg.data = message.index
                pub_waypoint_goto.publish(msg)
                rospy.loginfo("Ground-to-air: WaypointGoto")
            except:
                rospy.logwarn("Error processing command: WaypointGoto")
            
        elif isinstance(message, acs_messages.SlaveSetup):
            try:
                srv = rospy.ServiceProxy('autopilot/slave_setup', 
                                         apsrv.SlaveSetup)
                srv(message.enable, message.channel)
                rospy.loginfo("Ground-to-air: SlaveSetup")
            except Exception as ex:
                rospy.logwarn("Error processing command: SlaveSetup")
            
        elif isinstance(message, acs_messages.FlightReady):
            try:
                rospy.set_param("flight_ready", message.ready)
                rospy.loginfo("Ground-to-air: FlightReady")
            except Exception as ex:
                rospy.logwarn("Error processing command: FlightReady")

        elif isinstance(message, acs_messages.PayloadHeartbeat):
            try:
                srv = rospy.ServiceProxy('safety/set_health_state',
                                         ap_srvs.SetBoolean)
                srv(message.enable)
                rospy.loginfo("Ground-to-air: PayloadHeartbeat")
            except Exception as ex:
                rospy.logwarn("Error processing command: PayloadHeartbeat")
            
        elif isinstance(message, acs_messages.PayloadShutdown):
            try:
                res = os.system("sudo halt")
                if res != 0:
                    raise Exception
                rospy.loginfo("Ground-to-air: PayloadShutdown")
            except:
                rospy.logwarn("Error processing command: PayloadShutdown")
            
        
