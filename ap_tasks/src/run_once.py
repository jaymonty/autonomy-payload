#!/usr/bin/env python

#-----------------------------------------------------------------------
# Slate tasks to be run once certain events occur
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
import subprocess

# General ROS imports
import rospy

# Import ROS message and service types
from autopilot_bridge import msg as apmsg
from autopilot_bridge import srv as apsrv

#-----------------------------------------------------------------------
# Parameters

# Base name for node topics and services
ROS_BASENAME = 'run_once'

#-----------------------------------------------------------------------
# Various action functions

def ap_param_set():
    acid = rospy.get_param("aircraft_id")
    srv = rospy.ServiceProxy("autopilot/param_set", apsrv.ParamSet)
    res = srv("SYSID_THISMAV", float(acid))
    if not res.ok:
        raise Exception("Param set SYSID_THISMAV failed")

def rosbag_start():
    folder = '~/bags'
    acid = rospy.get_param("aircraft_id")
    subprocess.call("ls %s || mkdir %s" % (folder, folder), shell=True)
    rosbag_start.proc = subprocess.Popen("rosbag record -a -o %s/%u" % (folder, acid), shell=True)

def rosbag_stop():
    rosbag_start.proc.send_signal(subprocess.signal.SIGINT)

#-----------------------------------------------------------------------
# Tasks for when autopilot becomes "ready"

# Add functions to be run here
# If the function returns, it will be considered done and removed
#  from the list; if the function fails, it should raise an Exception
autopilot_ready_tasks = [ap_param_set, rosbag_start]

# This callback waits for the first 'status' message from the
# autopilot, indicating that there is positive communications
# between autopilot and payload, and that the payload clock
# has been set to GPS time.
def autopilot_ready_callback(data):
    global autopilot_ready_tasks

    for task in autopilot_ready_tasks:
        try:
            task()
            rospy.loginfo("Successfully completed autopilot_ready task")
            autopilot_ready_tasks.remove(task)
        except Exception as ex:
            rospy.logwarn("Failed to complete autopilot_ready task: " + ex.args[0])

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # ROS initialization
    rospy.init_node(ROS_BASENAME)
    
    # Set up ROS subscribers
    sub_ap_status = rospy.Subscriber("autopilot/status", apmsg.Status,
                                     autopilot_ready_callback)
    
    # Spin, responding to events
    rospy.spin()

    # Shut down anything as needed
    rosbag_stop()
