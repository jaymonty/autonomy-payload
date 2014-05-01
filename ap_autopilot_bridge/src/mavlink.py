#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-MAVLink Bridge node
# Mike Clement, 2014
#
# Some general info should go here!!
# Note: this is loosely based on, but heavily modified and extended
#   from, https://github.com/cberzan/roscopter

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
import datetime
from optparse import OptionParser
import os
import sys
import signal

# General ROS imports
#import roslib; roslib.load_manifest('ap_mavlink_bridge')
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import std_msgs.msg 
from tf.transformations import quaternion_from_euler

# Import ROS messages specific to this bridge
from ap_autopilot_bridge import msg as apmsg
from ap_safety_monitor import msg as safemsg

#-----------------------------------------------------------------------
# Parameters

# Base name for node topics and services
ROS_BASENAME = 'autopilot'

# Control printing of messages to stdout
DBUG_PRINT = False
WARN_PRINT = False

# Rate at which we request that mavlink sends its messages to us
MESSAGE_RATE = 10.0

# Rate at which we run the main loop (should be >= MESSAGE_RATE)
LOOP_RATE = MESSAGE_RATE

#-----------------------------------------------------------------------
# Ugly global variables

# Contains the mavlink 'master' object
master = None

# Last known custom_mode from AP (from last mavlink heartbeat msg)
ap_last_custom_mode = -1

# Track delta between AP's epoch time and local epoch time (in usec)
# (stored as GMT)
ap_time_delta = rospy.Duration(0, 0)

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
# Time functions

# Update delta between local and AP time (provided in usec)
def set_ap_time(ap_epoch_usec):
    global ap_time_delta
    if ap_epoch_usec == 0:  # If no AP time, ignore and use system time
        return
    ap_epoch_sec = int(ap_epoch_usec / 1e06)
    ap_nsec = (ap_epoch_usec % 1e06) * 1e03
    ap_time = rospy.Time(ap_epoch_sec, ap_nsec)
    ap_time_delta = rospy.Time.now() - ap_time

# Return projected time of AP (UTC) as rospy.Time object
def project_ap_time():
    global ap_time_delta
    return rospy.Time.now() - ap_time_delta

#-----------------------------------------------------------------------
# mavlink utility functions

def mavlink_setup(device, baudrate):
    global master
    global ap_last_custom_mode
    
    # Create 'master' (mavlink) object
    master = mavutil.mavlink_connection(device, baudrate)
    
    # Wait for a heartbeat so we know the target system IDs
    print("Waiting for AP heartbeat")
    msg = master.wait_heartbeat()
    ap_last_custom_mode = msg.custom_mode
    print("Heartbeat from AP (sys %u comp %u custom_mode %u)" %
          (master.target_system, master.target_system, ap_last_custom_mode))
    
    # Set up output stream from master
    print("Sending all stream request for rate %u" % MESSAGE_RATE)
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        MESSAGE_RATE,
        1)

#-----------------------------------------------------------------------
# ROS Subscriber callbacks

def log_rossub(data):
    log_dbug("ROSSUB (%s)" % data)

# Send an AP heartbeat whenever a heartbeast message comes in
def sub_heartbeat(data):
    if ap_last_custom_mode == -1:
        return
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, 
        mavutil.mavlink.MAV_TYPE_GENERIC, 
        0, 
        ap_last_custom_mode, 
        mavutil.mavlink.MAV_STATE_ACTIVE)
    log_rossub('heartbeat')

#-----------------------------------------------------------------------
# ROS services
# Just examples from roscopter for now

# These are examples left over from roscopter
def set_arm(req):
    master.arducopter_arm()
    return []

def set_disarm(req):
    master.arducopter_disarm()
    return []

#-----------------------------------------------------------------------
# Main Loop

def mainloop(opts):
    # ROS initialization
    rospy.init_node('mavlink')
    
    # Set up ROS publishers
    pub_gps = rospy.Publisher("%s/gps"%ROS_BASENAME, NavSatFix)
    pub_gps_odom = rospy.Publisher("%s/gps_odom"%ROS_BASENAME, Odometry)
    pub_imu = rospy.Publisher("%s/imu"%ROS_BASENAME, Imu)
    pub_status = rospy.Publisher("%s/status"%ROS_BASENAME, apmsg.Heartbeat)
    
    # Set up ROS subscribers
    rospy.Subscriber("safety/heartbeat", 
                     safemsg.Heartbeat, sub_heartbeat)
        
    # Set up ROS service callbacks
    #arm_service = rospy.Service('arm', Empty, set_arm)
    #disarm_service = rospy.Service('disarm', Empty, set_disarm)
    
    #<<< START LOOP INTERNALS >>>
    
    # Store "unknown" message types so we only warn once
    unknown_message_types = {}
    
    # Initialize mavlink connection
    mavlink_setup(opts.device, opts.baudrate)
    
    # Try to run this loop at LOOP_RATE Hz
    r = rospy.Rate(LOOP_RATE)
    print "\nStarting loop...\n"
    while not rospy.is_shutdown():
        # Process all new messages from master
        # Note: if too many messages come in, we'll never break
        #  out of this loop and subscribers/services won't run.
        #  Adjust loop-break conditions accordingly.
        while True:
            # Check for messages, break loop if none available
            msg = master.recv_match(blocking=False)
            if not msg:
                break
            
            # Look up type, ignore "bad data" messages
            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue
            
            # Message cases (observed from 'current' PX4 firmware)
            if msg_type == "AHRS":
                True
            elif msg_type == "AHRS2":
                True
            elif msg_type == "AIRSPEED_AUTOCAL":
                True
            elif msg_type == "ATTITUDE":
                imu = Imu()
                imu.header.stamp = project_ap_time()
                imu.header.frame_id = 'base_footprint'
                quat = quaternion_from_euler(msg.roll, msg.pitch, msg.yaw, 'sxyz')
                #print "R: %f P: %f Y: %f"%(msg.roll, msg.pitch, msg.yaw)
                #print "\t\t\t\tQ: %f, %f, %f, %f"%(quat[0], quat[1], quat[2], quat[3])
                imu.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
                pub_imu.publish(imu)
            elif msg_type == "FENCE_STATUS":
                True
            elif msg_type == "GLOBAL_POSITION_INT":
                # Publish a NavSatFix message
                fix = NavSatFix()
                fix.header.stamp = project_ap_time()
                fix.header.frame_id = 'base_footprint'
                fix.latitude = msg.lat/1e07
                fix.longitude = msg.lon/1e07
                fix.altitude = msg.alt/1e03
                fix.status.status = NavSatStatus.STATUS_FIX
                fix.status.service = NavSatStatus.SERVICE_GPS
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                pub_gps.publish(fix)
                
                # Also publish an Odometry message
                odom = Odometry()
                odom.header.stamp = project_ap_time()
                odom.header.frame_id = 'base_footprint'
                odom.pose.pose.position.x = msg.lat/1e07
                odom.pose.pose.position.y = msg.lon/1e07
                odom.pose.pose.position.z = msg.alt/1e03
                odom.pose.pose.orientation.x = 1
                odom.pose.pose.orientation.y = 0
                odom.pose.pose.orientation.z = 0
                odom.pose.pose.orientation.w = 0
                odom.pose.covariance = ( 0.1, 0, 0, 0, 0, 0,
                                         0, 0.1, 0, 0, 0, 0,
                                         0, 0, 0.1, 0, 0, 0,
                                         0, 0, 0, 99999, 0, 0,
                                         0, 0, 0, 0, 99999, 0,
                                         0, 0, 0, 0, 0, 99999 )
                pub_gps_odom.publish(odom)
            elif msg_type == "GPS_RAW_INT":
                True
            elif msg_type == "HEARTBEAT":
                pub_status.publish(
                    msg.base_mode & \
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 
                    msg.base_mode & \
                    mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 
                    mavutil.mode_string_v10(msg))
            elif msg_type == "HWSTATUS":
                True
            elif msg_type == "MEMINFO":
                True
            elif msg_type == "MISSION_CURRENT":
                True
            elif msg_type == "NAV_CONTROLLER_OUTPUT":
                True
            elif msg_type == "PARAM_VALUE":
                True
            elif msg_type == "POWER_STATUS":
                True
            elif msg_type == "RADIO":
                True
            elif msg_type == "RADIO_STATUS":
                True
            elif msg_type == "RAW_IMU" :
                True
            elif msg_type == "RC_CHANNELS_RAW":
                True
            elif msg_type == "SCALED_IMU2":
                True
            elif msg_type == "SCALED_PRESSURE":
                True
            elif msg_type == "SENSOR_OFFSETS":
                True
            elif msg_type == "SERVO_OUTPUT_RAW":
                True
            elif msg_type == "STATUS_TEXT":
                True
            elif msg_type == "STATUSTEXT":
                True
            elif msg_type == "SYS_STATUS":
                True
            elif msg_type == "SYSTEM_TIME":
                # Adjust known time offset from autopilot's
                set_ap_time(msg.time_unix_usec)
            elif msg_type == "VFR_HUD":
                True
            elif msg_type == "WIND":
                True
            else:
                # Report outliers so we can add them as new cases
                log_dbug("Unhandled message type %s" % msg_type)
                if msg_type not in unknown_message_types:
                    # Always report to debugging, but only warn once
                    unknown_message_types[msg_type] = True
                    log_warn("Unhandled message type %s" % msg_type)
                    continue
            
            # Report received message to console
            log_dbug("MAVLINK (%s)" % msg_type)
            
        # Sleep so ROS subscribers and services can run
        r.sleep()

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("bridge.py [options]")
    parser.add_option("--device", dest="device", 
                      help="serial device", default="auto-detect")
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="serial baud rate", default=57600)
    parser.add_option("--mavlinkdir", dest="mavlink_dir", 
                      help="path to mavlink folder", default=None)
    (opts, args) = parser.parse_args()
    
    # If device/baudrate are "auto-detect", try to figure them out
    if opts.device == "auto-detect":
        if os.path.exists("/dev/ttyACM0"):  # Indicates USB
            opts.device = "/dev/ttyACM0"
            opts.baudrate = 115200
        elif os.path.exists("/dev/ttyUSB0"):  # Indicates radio
            opts.device = "/dev/ttyUSB0"
        elif os.path.exists("/dev/ttyAMA0"):  # Indicates serial
            opts.device = "/dev/ttyAMA0"
        else:
            print "Error: could not find a suitable device.\n"
            sys.exit(-1)
    elif not os.path.exists(opts.device):
        print "Specified device doesn't exist.\n"
        sys.exit(-1)
    
    # User-friendly hello message
    print "Starting mavlink <-> ROS interface with these parameters:\n" + \
          ("  device:\t\t%s\n" % opts.device) + \
          ("  baudrate:\t\t%s\n" % str(opts.baudrate))
    
    # Allow adding custom lib path, in case mavlink isn't "installed"
    if opts.mavlink_dir:
        print "Adding custom mavlink path: %s\n" % opts.mavlink_dir
        sys.path.insert(0, opts.mavlink_dir)
    from pymavlink import mavutil
    
    # Everything else happens in mainloop()
    try:
        mainloop(opts)
    except rospy.ROSInterruptException: 
        pass


