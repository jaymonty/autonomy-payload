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

# Set system clock to provided epoch usecs
def set_system_time(epoch_usec):
    if not epoch_usec:
        log_warn("Requested system time invalid, using saved system time")
        return
    secs = int(epoch_usec / 1e06)
    nsecs = (epoch_usec % 1e06) * 1e03
    res = os.system("date -s '@%u.%u'" % (secs, nsecs))
    if res != 0:
        log_warn("Could not set system time, using saved system time")

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

def mavlink_setup(device, baudrate, skip_time_hack):
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
    
    # Wait for first *valid* SYSTEM_TIME message and set local clock
    if skip_time_hack:
        log_warn("Skipping time hack from autopilot, using saved system time")
        return
    print "Waiting for non-zero time hack from autopilot..."
    while True:
        msg = master.recv_match(type='SYSTEM_TIME', blocking=True)
        if msg.time_unix_usec:
            set_system_time(msg.time_unix_usec)
            break

def mavlink_sensor_health(bits):
    present = master.field('SYS_STATUS', 'onboard_control_sensors_enabled', 0)
    present = ((present & bits) == bits)
    healthy = master.field('SYS_STATUS', 'onboard_control_sensors_healthy', 0)
    healthy = ((healthy & bits) == bits)
    return (present and healthy)

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

def sub_guided_goto(data):
    if ap_last_custom_mode == -1:
        return
    self.master.mav.mission_item_send(self.target_system,
        self.target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 0, 0, 0, 0, 0,
        data.lat, data.lon, data.alt)
    log_rossub('guided_goto')

#-----------------------------------------------------------------------
# Main Loop

def mainloop(opts):
    # ROS initialization
    rospy.init_node('mavlink')
    
    # Set up ROS publishers
    pub_gps = rospy.Publisher("%s/gps"%ROS_BASENAME, NavSatFix)
    pub_gps_odom = rospy.Publisher("%s/gps_odom"%ROS_BASENAME, Odometry)
    pub_imu = rospy.Publisher("%s/imu"%ROS_BASENAME, Imu)
    pub_status = rospy.Publisher("%s/status"%ROS_BASENAME, apmsg.Status)
    
    # Set up ROS subscribers
    rospy.Subscriber("safety/heartbeat", 
                     safemsg.Heartbeat, sub_heartbeat)
    #rospy.Subscriber("%s/guided_goto"%ROS_BASENAME, apmsg.GuidedGoto, 
    #                 sub_guided_goto)
        
    #<<< START LOOP INTERNALS >>>
    
    # Initialize mavlink connection
    mavlink_setup(opts.device, opts.baudrate, opts.skip_time_hack)
    
    # Try to run this loop at LOOP_RATE Hz
    r = rospy.Rate(LOOP_RATE)
    print "\nStarting autopilot loop...\n"
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
            if msg_type == "ATTITUDE":
                imu = Imu()
                imu.header.stamp = project_ap_time()
                imu.header.frame_id = 'base_footprint'
                quat = quaternion_from_euler(msg.roll, msg.pitch, msg.yaw, 'sxyz')
                imu.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
                pub_imu.publish(imu)
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
            elif msg_type == "HEARTBEAT":
                sta = apmsg.Status()
                sta.header.stamp = project_ap_time()
                if master.flightmode == 'RTL':
                    sta.mode = sta.MODE_RALLY
                elif master.flightmode == 'MANUAL':
                    sta.mode = sta.MODE_MANUAL
                elif master.flightmode == 'FBWA':
                    sta.mode = sta.MODE_FBW
                elif master.flightmode == 'AUTO':
                    sta.mode = sta.MODE_AUTO
                elif master.flightmode == '':  # TODO: Define condition for 'landing'
                    sta.mode = sta.MODE_LAND
                else:
                    sta.mode = sta.MODE_UNKNOWN
                sta.armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                sta.ahrs_ok = mavlink_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_AHRS)
                sta.alt_rel = master.field('GLOBAL_POSITION_INT', 'relative_alt', 0)
                sta.as_ok = mavlink_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)
                sta.as_read = master.field('VFR_HUD', 'airspeed', 0)
                sta.gps_ok = (master.field('GPS_RAW_INT', 'fix_type', 0) == 3)
                sta.gps_sats = master.field('GPS_RAW_INT', 'satellites_visible', 0)
                sta.gps_eph = master.field('GPS_RAW_INT', 'eph', 0)
                sta.ins_ok = mavlink_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | \
                                                   mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO)
                sta.mag_ok = mavlink_sensor_health(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG)
                sta.mis_cur = master.field('MISSION_CURRENT', 'seq', 0)
                sta.pwr_ok = not (master.field('POWER_STATUS', 'flags', 0) \
                                & mavutil.mavlink.MAV_POWER_STATUS_CHANGED)
                sta.pwr_batt_rem = master.field('SYS_STATUS', 'battery_remaining', -1)
                sta.pwr_batt_vcc = master.field('SYS_STATUS', 'voltage_battery', -1)
                sta.pwr_batt_cur = master.field('SYS_STATUS', 'current_battery', -1)
                pub_status.publish(sta)
            elif msg_type == "SYSTEM_TIME":
                # Adjust known time offset from autopilot's
                #set_ap_time(msg.time_unix_usec)
                True
            
            # Report received message to console
            log_dbug("MAVLINK (%s)" % msg_type)
            
            # If you *really* want to see what's coming out
            if opts.spam_mavlink:
                print msg_type + " @ " + str(msg._timestamp) + ":\n  " \
                    + "\n  ".join("%s: %s" % (k, v) for (k, v) \
                                  in sorted(vars(msg).items()) \
                                  if not k.startswith('_'))
            
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
    parser.add_option("--skip-time-hack", dest="skip_time_hack", 
                      action="store_true", default=False)
    parser.add_option("--spam-mavlink", dest="spam_mavlink", 
                      action="store_true", default=False)
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
    elif opts.device.find(':') == -1 and not os.path.exists(opts.device):
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


