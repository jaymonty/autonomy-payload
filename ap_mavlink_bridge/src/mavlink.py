#!/usr/bin/env python

#-----------------------------------------------------------------------
# ROS-MAVLink Bridge node
# Mike Clement, 2014
#
# Some general info should go here!!
# Note: this is currently just a refactored version of roscopter
#   as cloned from https://github.com/cberzan/roscopter

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
import os
import sys
import struct
import time
from optparse import OptionParser

# General ROS imports
#import roslib; roslib.load_manifest('ap_mavlink_bridge')
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

# Import ROS messages specific to this bridge
#import ap_mavlink_bridge.msg
import ap_mavlink_bridge.msg

#-----------------------------------------------------------------------
# Ugly global data

master = None


#-----------------------------------------------------------------------
# mavlink utility functions

def mavlink_setup(device, baudrate, msgrate):
    global master
    
    # Create 'master' (mavlink) object
    master = mavutil.mavlink_connection(device, baudrate)
    
    # Wait for a heartbeat so we know the target system IDs
    print("Waiting for APM heartbeat")
    master.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" \
          % (master.target_system, master.target_system))
    
    # Set up output stream from master
    print("Sending all stream request for rate %u" % msgrate)
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        msgrate,
        1)

#-----------------------------------------------------------------------
# ROS Subscriber callbacks

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
    print "sending rc: %s" % data

#-----------------------------------------------------------------------
# ROS services

def set_arm(req):
    master.arducopter_arm()
    return []

def set_disarm(req):
    master.arducopter_disarm()
    return []

#-----------------------------------------------------------------------
# Main Loop

def mainloop(opts):
    # Initialize mavlink connection
    mavlink_setup(opts.device, opts.baudrate, opts.msgrate)
    
    # ROS initialization
    rospy.init_node('ap_mavlink_bridge')
    
    # Set up ROS publishers
    pub_gps = rospy.Publisher('gps', NavSatFix)
    pub_rc = rospy.Publisher('rc', ap_mavlink_bridge.msg.RC)
    pub_state = rospy.Publisher('state', ap_mavlink_bridge.msg.State)
    pub_vfr_hud = rospy.Publisher('vfr_hud', ap_mavlink_bridge.msg.VFR_HUD)
    pub_attitude = rospy.Publisher('attitude', ap_mavlink_bridge.msg.Attitude)
    pub_raw_imu = rospy.Publisher('raw_imu', \
                                  ap_mavlink_bridge.msg.Mavlink_RAW_IMU)
    pub_debug = rospy.Publisher('debug', String)
    
    # Set up ROS subscribers
    rospy.Subscriber("send_rc", ap_mavlink_bridge.msg.RC, send_rc)
        
    # Set up ROS service callbacks
    arm_service = rospy.Service('arm', Empty, set_arm)
    disarm_service = rospy.Service('disarm', Empty, set_disarm)
    
    # <<< BEGIN MAIN LOOP >>>
    
    # Try to loop at same rate as we request messages from master
    r = rospy.Rate(opts.msgrate)
    while not rospy.is_shutdown():
        # Process all new messages from master
        # (May need to adjust this; if messages come in too fast,
        #  won't ever get to other parts of the loop!)
        while True:
            # Check for messages, break loop if none available
            msg = master.recv_match(blocking=False)
            if not msg:
                break
            msg_type = msg.get_type()
            
            # Message cases (so far, copied from roscopter)
            if msg_type == "BAD_DATA":
                continue
            elif msg_type == "RC_CHANNELS_RAW":
                pub_rc.publish([msg.chan1_raw, msg.chan2_raw, \
                                msg.chan3_raw, msg.chan4_raw, \
                                msg.chan5_raw, msg.chan6_raw, \
                                msg.chan7_raw, msg.chan8_raw]) 
            elif msg_type == "HEARTBEAT":
                pub_state.publish(msg.base_mode & \
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 
                                  msg.base_mode & \
                                 mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 
                                  mavutil.mode_string_v10(msg))
            elif msg_type == "VFR_HUD":
                pub_vfr_hud.publish(msg.airspeed, msg.groundspeed, \
                                    msg.heading, msg.throttle, \
                                    msg.alt, msg.climb)
            elif msg_type == "GPS_RAW_INT":
                fix = NavSatStatus.STATUS_NO_FIX
                if msg.fix_type >=3:
                    fix = NavSatStatus.STATUS_FIX
                ns_status = NavSatStatus(status=fix, \
                                         service = NavSatStatus.SERVICE_GPS)
                pub_gps.publish(NavSatFix(latitude = msg.lat/1e07, \
                                          longitude = msg.lon/1e07, \
                                          altitude = msg.alt/1e03, \
                                          status = ns_status))
            elif msg_type == "ATTITUDE" :
                pub_attitude.publish(msg.roll, msg.pitch, \
                                     msg.yaw, msg.rollspeed, \
                                     msg.pitchspeed, msg.yawspeed)
            elif msg_type == "LOCAL_POSITION_NED" :
                print "Local Pos: (%f %f %f) , (%f %f %f)" \
                    % (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
            elif msg_type == "RAW_IMU" :
                pub_raw_imu.publish(Header(), msg.time_usec, \
                                    msg.xacc, msg.yacc, msg.zacc, \
                                    msg.xgyro, msg.ygyro, msg.zgyro, \
                                    msg.xmag, msg.ymag, msg.zmag)
            else:
                if True:
                    print "Unhandled message type %s" % msg_type
        
        # Handle any periodic tasks
        # Should see this message at ~10 Hz, or we're trying to do
        #   too much per big loop iteration
        pub_debug.publish("periodic")
        
        # Sleep so ROS subscribers and services can run
        r.sleep()

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # Default path to mavlink (based on where maday puts it)
    mavlink_dir = os.path.expandvars("/home/$USER/virtPlane/mavlink")
    
    # Grok args
    parser = OptionParser("ap_mavlink_bridge.py [options]")
    parser.add_option("--device", dest="device", \
                      help="serial device", default="/dev/ttyUSB0")
    parser.add_option("--baudrate", dest="baudrate", type='int', \
                      help="master port baud rate", default=57600)
    parser.add_option("--msgrate", dest="msgrate", type='int', \
                      help="requested stream rate", default=10)
    parser.add_option("--mavlinkdir", dest="mavlink_dir", \
                      help="path to mavlink folder", \
                      default=mavlink_dir)
    (opts, args) = parser.parse_args()
    
    # User-friendly hello message
    print "Starting mavlink <-> ROS interface with these parameters:\n" \
        + ("  device:\t\t%s\n" % opts.device) \
        + ("  baudrate:\t\t%s\n" % str(opts.baudrate)) \
        + ("  message rate:\t\t%s\n" % str(opts.msgrate)) \
        + ("  mavlink path:\t\t%s\n" % str(opts.mavlink_dir))
    
    # Import pymavlink
    sys.path.insert(0, opts.mavlink_dir)
    sys.path.insert(0, os.path.join(opts.mavlink_dir, 'pymavlink'))
    import mavutil
    
    # Everything else happens in mainloop()
    try:
        mainloop(opts)
    except rospy.ROSInterruptException: 
        pass


