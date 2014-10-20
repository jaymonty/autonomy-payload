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

# Use rosbag's /clock topic
def clock_callback(msg):
    global last_clock, last_local

    last_clock = msg.clock.secs + msg.clock.nsecs / 1e9
    last_local = time.time()

def net_callback(msg):
    global last_clock, last_local, aircraft_id, last_latency

    # Time the message was received (projected)
    rcv_time = last_clock + time.time() - last_local

    # CHOOSE ONE OF THE FOLLOWING (COMMENT OUT THE OTHER)
    # Evaluate latency from original data until /network/recv_pose
    msg_time = msg.pose.header.stamp.secs + msg.pose.header.stamp.nsecs / 1e9
    # Evaluate latency from original data until /local_estim/odom_combined
#    msg_time = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9

    # Calculate the actual latency
    latency = rcv_time - msg_time

    pstr = ("%10.09f" % msg_time) + ' -> ' + ("%10.09f" % rcv_time) + ' : ' + str(latency)

    # Using '\b' to erase so we're just printing to the same line over and over
    sys.stdout.write('\b' * len(last_latency) + pstr)
    sys.stdout.flush()
    last_latency = pstr

if __name__ == '__main__':
    rospy.init_node("latency")
    rospy.Subscriber("clock", Clock, clock_callback)

    # CHOOSE ONE OF THE FOLLOWING (COMMENT OUT THE OTHER)
    # Evaluate /network/recv_pose
    rospy.Subscriber("network/recv_pose", NetPoseStamped, net_callback)
    # Evaluate /local_estim/odom_combined
#    rospy.Subscriber("local_estim/odom_combined", PoseWithCovarianceStamped, net_callback)

    rospy.spin()
