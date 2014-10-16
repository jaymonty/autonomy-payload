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
from autopilot_bridge import msg as apmsg
from ap_srvs import srv as ap_srvs

#-----------------------------------------------------------------------
# Parameters

# Base name for node topics and services
ROS_BASENAME = 'safety'

#-----------------------------------------------------------------------
# Ugly global variables

# Overall, is the system healthy and should we send heartbeats?
all_is_well = True

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
                                    apmsg.Heartbeat)
    
    # Set up ROS subscribers
    
    
    # Set up ROS services
    rospy.Service("%s/set_health_state"%ROS_BASENAME,
                  ap_srvs.SetBoolean, svc_set_health)
    
    # Start main loop
    hb = apmsg.Heartbeat()
    r = rospy.Rate(1.0)
    print "\nStarting safety monitor loop...\n"
    while not rospy.is_shutdown():
        # Publish a heartbeat, if all is well
        if all_is_well:
            hb.counter = next_counter()
            pub_heartbeat.publish(hb)
        
        # Sleep so ROS subscribers and services can run
        r.sleep()


