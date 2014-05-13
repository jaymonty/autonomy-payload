#!/usr/bin/env python

#-----------------------------------------------------------------------
# Network pose data logging node
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
from optparse import OptionParser

# General ROS imports
import rospy

# Import ROS message and service types
from sensor_msgs.msg import NavSatFix
from ap_autopilot_bridge import msg as apmsg
from ap_network_bridge import msg as netmsg

#-----------------------------------------------------------------------
# Ugly global variables

ap_time_delta = rospy.Duration(0, 0)
log_file = None

#-----------------------------------------------------------------------
# ROS Subscriber callbacks

def sub_ap_gps(data):
    # Update notion of local autopilot time
    global ap_time_delta
    ap_time_delta = rospy.Time.now() - data.header.stamp

def sub_net_pose(data):
    global ap_time_delta, log_file
    local_time = rospy.Time.now() - ap_time_delta
    net_id = data.sender_id
    net_time = data.pose.header.stamp
    if log_file is not None:
        log_file.write("ID %d SEND_TIME %u.09%u RECV_TIME %u.09%u\n" % \
                       (net_id, net_time.secs, net_time.nsecs,
                        local_time.secs, local_time.nsecs))
        log_file.flush()

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("net_pose.py [options]")
    parser.add_option("--file", dest="filename", 
                      help="File to log to", default="")
    (opts, args) = parser.parse_args()
    
    # ROS initialization
    rospy.init_node('log_net_pose')
    
    # Set up ROS subscribers
    rospy.Subscriber("autopilot/gps", NavSatFix, sub_ap_gps)
    rospy.Subscriber("network/pose", netmsg.NetPoseStamped, sub_net_pose)
    
    log_file = None
    if opts.filename != "":
        log_file = open(opts.filename, "a+")
        log_file.write("START\n")
        log_file.flush()
    
    # Start main loop
    print "\nStarting network pose logger...\n"
    rospy.spin()

