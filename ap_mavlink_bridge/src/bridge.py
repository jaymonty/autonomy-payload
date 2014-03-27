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
import os
import sys
import signal
from optparse import OptionParser

# General ROS imports
#import roslib; roslib.load_manifest('ap_mavlink_bridge')
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import Empty
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

# Rate at which we request that mavlink sends its messages to us
MESSAGE_RATE = 10.0

# Rate at which we run the main loop (should be >= MESSAGE_RATE)
LOOP_RATE = MESSAGE_RATE

#-----------------------------------------------------------------------
# Ugly global variables

# Contains the mavlink 'master' object
master = None

# Timer object used for periodic_run
periodic_timer = None

# Dictionary of periodic tasks
# 'task name' -> (callback, ticks_period, ticks_left)
# ticks are in increments of 1/LOOP_RATE seconds
periodic_tasks = {}

# Last known custom_mode from AP (from last heartbeat)
ap_last_custom_mode = -1

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
# Periodic task management
# (This could all be replaced with rospy.Timer if we wanted,
#  but it's nice to have some explicit control)

# Create a new task called 'name' to run 'callback' at 'hz' Hz
# Note: can't run a task faster than LOOP_RATE Hz
def periodic_new(name, callback, hz):
    ticks = max(1, int(LOOP_RATE / hz))
    periodic_tasks[name] = (callback, ticks, ticks)

# This runs the tasks on schedule, and is called periodically
#  by a rospy.Timer instance (see main loop)
def periodic_run(timer_event):
    # Publish so we can check that rates are being upheld
    periodic_run.pub.publish("rate check")
    
    # Go through periodically-scheduled tasks
    for task in periodic_tasks.keys():
        (cb, period, left) = periodic_tasks[task]
        # Check if the task is due; if so, run it!
        left -= 1
        if (left == 0):
            log_dbug("PERIODIC (%s)" % task)
            cb()
            left = period
        periodic_tasks[task] = (cb, period, left)
periodic_run.pub = rospy.Publisher("%s/ratecheck"%ROS_BASENAME, String)


#-----------------------------------------------------------------------
# ROS Subscriber callbacks
# Just examples from roscopter for now

def log_rossub(data):
    log_dbug("ROSSUB (%s)" % data)

# This is an example left over from roscopter
def send_rc(data):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        data.channel[0],
        data.channel[1],
        data.channel[2],
        data.channel[3],
        data.channel[4],
        data.channel[5],
        data.channel[6],
        data.channel[7])
    log_rossub('send_rc')

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
# Periodic callbacks

# Send heartbeat to AP so it doesn't RTL
def send_heartbeat():
    if ap_last_custom_mode == -1:
        return
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, 
        mavutil.mavlink.MAV_TYPE_GENERIC, 
        0, 
        ap_last_custom_mode, 
        mavutil.mavlink.MAV_STATE_ACTIVE)

#-----------------------------------------------------------------------
# Main Loop

def on_ros_shutdown():
    # Turn off periodic tasks
    global periodic_timer
    if (periodic_timer):
        periodic_timer.shutdown()

def mainloop(opts):
    # ROS initialization
    rospy.init_node('ap_mavlink_bridge')
    rospy.on_shutdown(on_ros_shutdown)
    
    # Set up ROS publishers
    # TODO: Configure publishers using a list of tuples of
    #  (mavlink_message_type, ros_topic_name, ros_topic_type)
    #  and build a dictionary of publishers at startup s.t.
    #  pub_dict[mavlink_message_type] == configured_publisher
    pub_ahrs        = None
    pub_attitude    = rospy.Publisher("%s/attitude"%ROS_BASENAME, 
                                      mavmsg.Attitude)
    pub_glo_pos_int = None
    pub_gps_raw_int = rospy.Publisher("%s/gps_raw"%ROS_BASENAME, 
                                      NavSatFix)
    pub_heartbeat   = rospy.Publisher("%s/heartbeat"%ROS_BASENAME, 
                                      mavmsg.Heartbeat)
    pub_hwstatus    = None
    pub_meminfo     = None
    pub_mis_cur     = None
    pub_nav_con_out = None
    pub_radio       = None
    pub_raw_imu     = rospy.Publisher("%s/raw_imu"%ROS_BASENAME, 
                                      mavmsg.RawIMU)
    pub_rc_chan_raw = rospy.Publisher("%s/rc_chan"%ROS_BASENAME, 
                                      mavmsg.RCRaw)
    pub_scaled_imu  = None
    pub_scaled_pres = None
    pub_sens_off    = None
    pub_ser_out_raw = None
    pub_sys_status  = None
    pub_sys_time    = None
    pub_vfr_hud     = rospy.Publisher("%s/vfr_hud"%ROS_BASENAME, 
                                      mavmsg.VFR_HUD)
    pub_wind        = None
    
    # Set up ROS subscribers
    #rospy.Subscriber("send_rc", ap_mavlink_bridge.msg.RC, send_rc)
        
    # Set up ROS service callbacks
    #arm_service = rospy.Service('arm', Empty, set_arm)
    #disarm_service = rospy.Service('disarm', Empty, set_disarm)
    
    # Set up periodic tasks
    periodic_new("heartbeat", send_heartbeat, 1.0)
    
    #<<< START LOOP INTERNALS >>>
    
    # Initialize mavlink connection
    mavlink_setup(opts.device, opts.baudrate)
    
    # Start running periodic task scheduler at LOOP_RATE Hz
    global periodic_timer 
    periodic_timer = \
        rospy.Timer(rospy.Duration(1.0/float(LOOP_RATE)), periodic_run)
    
    # Try to run this loop at LOOP_RATE Hz, but since we enforce
    #  periodic tasks using a ROS timer, it's okay to lag
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
            # TODO: Following creation of publisher dictionary above,
            #  replace cases with a single lookup and function call.
            #  (Note, will need a way to map msg.* to topic elements)
            if msg_type == "AHRS":
                pub_ahrs = None
            elif msg_type == "ATTITUDE" :
                pub_attitude.publish(msg.roll, msg.pitch, 
                                     msg.yaw, msg.rollspeed, 
                                     msg.pitchspeed, msg.yawspeed)
            elif msg_type == "GLOBAL_POSITION_INT":
                pub_glo_pos_int = None
            elif msg_type == "GPS_RAW_INT":
                fix = NavSatStatus.STATUS_NO_FIX
                if msg.fix_type >=3:
                    fix = NavSatStatus.STATUS_FIX
                ns_status = NavSatStatus(
                                status=fix, 
                                service = NavSatStatus.SERVICE_GPS)
                pub_gps_raw_int.publish(NavSatFix(latitude = msg.lat/1e07,
                                                  longitude = msg.lon/1e07,
                                                  altitude = msg.alt/1e03, 
                                                  status = ns_status))
            elif msg_type == "HEARTBEAT":
                pub_heartbeat.publish(
                    msg.base_mode & \
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 
                    msg.base_mode & \
                    mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 
                    mavutil.mode_string_v10(msg))
            elif msg_type == "HWSTATUS":
                pub_hwstatus = None
            elif msg_type == "MEMINFO":
                pub_meminfo = None
            elif msg_type == "MISSION_CURRENT":
                pub_mis_cur = None
            elif msg_type == "NAV_CONTROLLER_OUTPUT":
                pub_nav_con_out = None
            elif msg_type == "RADIO":
                pub_raio = None
            elif msg_type == "RAW_IMU" :
                pub_raw_imu.publish(Header(), msg.time_usec, 
                                    msg.xacc, msg.yacc, msg.zacc, 
                                    msg.xgyro, msg.ygyro, msg.zgyro, 
                                    msg.xmag, msg.ymag, msg.zmag)
            elif msg_type == "RC_CHANNELS_RAW":
                pub_rc_chan_raw.publish([msg.chan1_raw, msg.chan2_raw, 
                                         msg.chan3_raw, msg.chan4_raw, 
                                         msg.chan5_raw, msg.chan6_raw, 
                                         msg.chan7_raw, msg.chan8_raw]) 
            elif msg_type == "SCALED_IMU2":
                pub_scaled_imu = None
            elif msg_type == "SCALED_PRESSURE":
                pub_scaled_pres = None
            elif msg_type == "SENSOR_OFFSETS":
                pub_sens_off = None
            elif msg_type == "SERVO_OUTPUT_RAW":
                pub_ser_out_raw = None
            elif msg_type == "SYS_STATUS":
                pub_sys_status = None
            elif msg_type == "SYSTEM_TIME":
                pub_sys_time = None
            elif msg_type == "VFR_HUD":
                pub_vfr_hud.publish(msg.airspeed, msg.groundspeed, 
                                    msg.heading, msg.throttle, 
                                    msg.alt, msg.climb)
            elif msg_type == "WIND":
                pub_wind = None
            else:
                # Report outliers so we can add them
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
    parser = OptionParser("ap_mavlink_bridge.py [options]")
    parser.add_option("--device", dest="device", 
                      help="serial device", default="/dev/ttyUSB0")
    parser.add_option("--baudrate", dest="baudrate", type='int', 
                      help="master port baud rate", default=57600)
    parser.add_option("--mavlinkdir", dest="mavlink_dir", 
                      help="path to mavlink folder", 
                      default=None)
    (opts, args) = parser.parse_args()
    
    # User-friendly hello message
    print "Starting mavlink <-> ROS interface with these parameters:\n" + \
          ("  device:\t\t%s\n" % opts.device) + \
          ("  baudrate:\t\t%s\n" % str(opts.baudrate))
    
    # Import mavlink
    # (Allow adding custom lib path, in case mavlink isn't "installed")
    if opts.mavlink_dir:
        print "Adding custom mavlink path: %s\n" % opts.mavlink_dir
        sys.path.insert(0, opts.mavlink_dir)
    from pymavlink import mavutil
    
    # Everything else happens in mainloop()
    try:
        mainloop(opts)
    except rospy.ROSInterruptException: 
        pass


