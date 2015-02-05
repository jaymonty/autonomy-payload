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
    if not hasattr(rosbag_start, 'proc'):
        return
    rosbag_start.proc.send_signal(subprocess.signal.SIGINT)

#-----------------------------------------------------------------------
# Task iteration loop
# Take a list of task functions and optional a 'type' (name to print)
# Return those functions that threw exceptions

def attempt_tasks(task_list, task_type="\b"):
    tasks_remaining = task_list

    for task in tasks_remaining:
        try:
            task()
            rospy.loginfo("Successfully completed %s task" % task_type)
            tasks_remaining.remove(task)
        except Exception as ex:
            rospy.logwarn("Failed to complete %s task: %s" % (task_type, ex.args[0]))

    return tasks_remaining

#-----------------------------------------------------------------------
# Task callbacks

# This callback waits for the first 'status' message from the
# autopilot, indicating that there is positive communications
# between autopilot and payload, and that the payload clock
# has been set to GPS time.
def autopilot_ready_callback(data):
    global autopilot_ready_tasks

    # Updating the list causes a retry on subsequent callback firings
    autopilot_ready_tasks = attempt_tasks(autopilot_ready_tasks, "autopilot_ready")

# This callback gets fired when the ROS node is shut down
def ros_shutdown_callback():
    global ros_shutdown_tasks
    attempt_tasks(ros_shutdown_tasks, "ros_shutdown")

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # ROS initialization
    rospy.init_node(ROS_BASENAME)
    
    # Set up lists of tasks to complete
    autopilot_ready_tasks = [ap_param_set]
    ros_shutdown_tasks = []

    # Conditionally add tasks
    if rospy.has_param('rosbag_enable') and rospy.get_param('rosbag_enable'):
        autopilot_ready_tasks.append(rosbag_start)
        ros_shutdown_tasks.append(rosbag_stop)

    # Set up ROS subscribers
    sub_ap_status = rospy.Subscriber("autopilot/status", apmsg.Status,
                                     autopilot_ready_callback)
    
    # Spin, responding to events
    rospy.spin()

    # Shut down anything as needed
    ros_shutdown_callback()

