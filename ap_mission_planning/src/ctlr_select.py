#!/usr/bin/env python

#-----------------------------------------------------------------------
# Controller-Selector Node
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

import sys

import rospy

import std_msgs.msg as std_msgs
import ap_msgs.msg as payload_msgs
import autopilot_bridge.msg as mavbridge_msgs
from ap_lib import controller

#-----------------------------------------------------------------------
# Class definitions

class ControllerType(object):
    def __init__(self, c_id, c_name, c_topic_base):
        self.id = c_id
        self.name = c_name

        self._run_pub = rospy.Publisher("%s/%s_run" % (c_topic_base, c_name),
                                        std_msgs.Bool)

        self.status = payload_msgs.ControllerState()
        self.status.controller_id = c_id
        self.status.sequence = 0
        self.status.is_ready = False
        self.status.is_active = False

        self.status_last = rospy.Time()

    def update_status(self, status):
        if status.controller_id != self.id:
            raise Exception("Cannot update status with mismatched id")
        self.status = status
        self.status_last = rospy.Time.now()

    def set_active(self, active):
        msg = std_msgs.Bool()
        msg.data = active
        self._run_pub.Publish(msg)

class ControllerSelector(object):
    ros_basename = 'ctlr_select'

    def __init__(self,
                 basename='ctlr_select',  # TODO: sync w class variable
                 loiter_wp='payload_loiter_wp'):
        self._basename = basename
        self._loiter_wp_param = loiter_wp

        self._controllers = {}
        self._current_mode = 0

        self._ap_status = None
        self._ap_status_last = rospy.Time()

#        self._pub_status = rospy.Publisher("%s/selector_status" % basename,
#                                           createmessagetype)
        self._pub_wpindex = rospy.Publisher("%s/autopilot_wpindex" % basename,
                                            std_msgs.UInt16)

        rospy.Subscriber("%s/controller_status" % basename,
                         payload_msgs.ControllerState,
                         self._sub_ctlr_status)
        rospy.Subscriber("%s/selector_mode" % basename,
                         std_msgs.UInt8,
                         self._sub_ctlr_mode)
        rospy.Subscriber("%s/autopilot_status" % basename,
                         mavbridge_msgs.Status,
                         self._sub_ap_status)
                        

    def add_controller(self, c_id, c_name):
        if c_id in self._controllers:
            raise Exception("Attempted to redefine controller %u (%s) as %s" % \
                            (c_id, self.controllers[c_id].name, c_name))

        try:
            self._controllers[c_id] = ControllerType(c_id, c_name, self._basename)
        except Exception as ex:
            raise Exception("Failed to define controller %u (%s): %s" % \
                            (c_id, c_name, ex.args[0]))

    def _sub_ctlr_mode(self, msg):
        mode = msg.data
        stale_time = rospy.Time.now() - rospy.Duration(3)

        try:
            # Is requested mode known to us?
            if mode not in self._controllers:
                raise Exception("Unknown controller type")
    
            # Have we heard from the autopilot recently?
            if self._ap_status is None or \
               self._ap_status_last < stale_time:
                raise Exception("Unknown or stale autopilot status")
    
            # Is the autopilot in AUTO?
            if self._ap_status.mode != mavbridge_msgs.Status.MODE_AUTO:
                raise Exception("Autopilot not in AUTO")
    
            # Have we heard from the requested controller recently?
            if self._controllers[mode].status_last < stale_time:
                raise Exception("Unknown or stale controller status")
    
            # Is the controller ready?
            if not self._controllers[mode].status.is_ready:
                raise Exception("Requested controller is not ready")
    
            # Is the controller not yet active?
            if self._controllers[mode].status.is_active:
                # NOTE: Not really an error, but we'll report it anyway
                raise Exception("Requested controller is already active")
    
            # Get inifinite loiter waypoint index
            if not rospy.has_param(self._loiter_wp_param):
                raise Exception("Infinite loiter waypoint parameter not defined")
            try:
                loiter_wp = rospy.get_param(self._loiter_wp_param)
            except Exception as ex:
                raise Exception("Error getting infinite waypoint index: %s" % ex.args[0])
    
            # Shut down any other active controllers
            for c in self._controllers:
                if self._controllers[c].status.is_active:
                    self._controllers[c].set_active(False)
    
            # Make sure the autopilot is using the infinite loiter waypoint
            if self._ap_status.mode != loiter_wp:
                wpindex = std_msgs.UInt16()
                wpindex.data = loiter_wp
                self._pub_wpindex.Publish(wpindex)
                # TODO: loop, sleeping and checking that index took,
                # or timing out and recovering
    
            # Activate controller
            self._controllers[mode].set_active(True)
            # TODO: loop, sleeping and checking that mode took,
            # or timing out and recovering

            # Update state
            self._current_mode = mode

        except Exception as ex:
            rospy.log_warn("Set Control Mode: " + ex.args[0])
            # TODO: Implement "safe" recovery into known state

    def _sub_ctlr_status(self, msg):
        if msg.controller_id not in self._controllers:
            raise Exception("Received status message from unknown controller type")
        self._controllers[msg.controller_id].update_status(msg)

    def _sub_ap_status(self, msg):
        self._ap_status = msg
        self._ap_status_last = rospy.Time.now()

    def loop(self, rate):
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            # Publish selector status message
#            status = createmessagetype()
#            status.a_field = None
#            self._pub_status.Publish(status)

            # Sleep then iterate
            r.sleep()

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # ROS init
    rospy.init_node(ControllerSelector.ros_basename)

    # Initialize state machine
    ctlrsel = ControllerSelector()
    ctlrsel.add_controller(controller.WP_SEQUENCE_CTRLR, "wpsequencer")
    ctlrsel.add_controller(controller.FOLLOW_CTRLR, "follower")

    # Start loop
    ctlrsel.loop(1.0)

