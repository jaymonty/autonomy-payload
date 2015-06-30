#!/usr/bin/env python

#-----------------------------------------------------------------------
# Aggregate and summarize /network/recv_pose message stats
# Mike Clement, 2015
#
# Some general info should go here!!

import rospy

from autopilot_bridge import msg as pilot_msg
from ap_msgs import msg as ap_msg

#-----------------------------------------------------------------------
# State

msg_rates = {}

#-----------------------------------------------------------------------
# Callbacks

def sub_recv_pose(msg):
    if msg.vehicle_id not in msg_rates:
        msg_rates[msg.vehicle_id] = ap_msg.MsgStat()
        msg_rates[msg.vehicle_id].id = msg.vehicle_id

    # Increment count of messages seen this period
    msg_rates[msg.vehicle_id].count += 1

    # Accumulate latency (will be averaged during re-publish)
    time_sent = msg.state.header.stamp
    time_rcvd = msg.received_at
    latency = float(str(time_rcvd - time_sent)) / 1e9
    msg_rates[msg.vehicle_id].latency += latency

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # ROS initialization
    rospy.init_node('recv_pose_rate')

    # Set up ROS subscribers
    rospy.Subscriber("network/recv_pose", ap_msg.SwarmVehicleState,
                     sub_recv_pose)

    # Set up ROS publishers
    pose_pub = rospy.Publisher("logging/recv_pose_rate", ap_msg.MsgStatList,
                               queue_size=1)

    # Start main loop
    print "\nStarting recv_pose_rate node ...\n"
    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        # Re-publish data at this rate
        msg = ap_msg.MsgStatList()
        msg.header.stamp = rospy.Time.now()
        for k,v in sorted(msg_rates.items()):
            v.latency /= float(v.count)
            msg.stat.append(v)
        pose_pub.publish(msg)

        # Clear data for next sample period
        msg_rates = {}

        # Sleep, just servicing callbacks
        r.sleep()
