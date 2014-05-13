#!/usr/bin/env python

#-----------------------------------------------------------------------
# Flight safety and aircraft health node
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports


# General ROS imports
import rospy

# Import ROS message and service types
import std_msgs.msg
import std_srvs.srv
from ap_safety_monitor import msg as safemsg
from ap_safety_monitor import srv as safesrv

#-----------------------------------------------------------------------
# Parameters

# Base name for node topics and services
ROS_BASENAME = 'safety'

# Control printing of messages to stdout
DBUG_PRINT = False
WARN_PRINT = False

#-----------------------------------------------------------------------
# Ugly global variables

# Overall, is the system healthy and should we send heartbeats?
all_is_well = True

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
# ROS Subscriber callbacks



#-----------------------------------------------------------------------
# ROS services

# Enable or disable heartbeat, depending on data.enable
def svc_set_health(data):
    global all_is_well
    if data.enable == 0:
        all_is_well = False
    else:
        all_is_well = True
    return 1

#-----------------------------------------------------------------------
# Manage heartbeat counter

hb_counter = 0

def next_counter():
    global hb_counter
    hb_counter += 1
    return hb_counter


#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # ROS initialization
    rospy.init_node('ap_safety_monitor')
    
    # Set up ROS publishers
    pub_heartbeat = rospy.Publisher("%s/heartbeat"%ROS_BASENAME,
                                    safemsg.Heartbeat)
    
    # Set up ROS subscribers
    
    
    # Set up ROS services
    rospy.Service("%s/set_health"%ROS_BASENAME,
                  safesrv.SetHealth, svc_set_health)
    
    # Start main loop
    hb = safemsg.Heartbeat()
    r = rospy.Rate(1.0)
    print "\nStarting safety monitor loop...\n"
    while not rospy.is_shutdown():
        # Publish a heartbeat, if all is well
        if all_is_well:
            hb.counter = next_counter()
            pub_heartbeat.publish(hb)
        
        # Sleep so ROS subscribers and services can run
        r.sleep()


