#!/usr/bin/env python

# Get a sense of message latency from generation to reception

import rospy
import sys
import time

from ap_network_bridge.msg import NetPoseStamped
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped

last_clock = 0.0
last_local = 0.0

last_latency = ''

def clock_callback(msg):
    global last_clock, last_local

    last_clock = msg.clock.secs + msg.clock.nsecs / 1e9
    last_local = time.time()

def net_callback(msg):
    global last_clock, last_local, aircraft_id, last_latency

    rcv_time = last_clock + time.time() - last_local
    # Evaluate /network/recv_pose
    msg_time = msg.pose.header.stamp.secs + msg.pose.header.stamp.nsecs / 1e9
    # Evaluate /local_estim/odom_combined
#    msg_time = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9
    latency = rcv_time - msg_time

    pstr = ("%10.09f" % msg_time) + ' -> ' + ("%10.09f" % rcv_time) + ' : ' + str(latency)

    sys.stdout.write('\b' * len(last_latency) + pstr)
    sys.stdout.flush()
    last_latency = pstr

if __name__ == '__main__':
    rospy.init_node("latency")
    rospy.Subscriber("clock", Clock, clock_callback)
    # Evaluate /network/recv_pose
    rospy.Subscriber("network/recv_pose", NetPoseStamped, net_callback)
    # Evaluate /local_estim/odom_combined
#    rospy.Subscriber("local_estim/odom_combined", PoseWithCovarianceStamped, net_callback)
    rospy.spin()
