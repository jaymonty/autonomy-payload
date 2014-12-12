#!/usr/bin/env python

#-------------------------------------------------------------------------
# Nodeable
# Duane Davis, 2014
#
# Defines an abstract class for wrapping ACS controllers that work by
# computing waypoints into ROS objects that can be incorporated into
# other classes or run as independent nodes
#-------------------------------------------------------------------------

# ROS imports
import rospy
from rospy import rostime
from ap_lib import nodeable
from ap_lib import controller
from autopilot_bridge.msg import LLA


# Global variables (constants)
MIN_REL_ALT = 50.0 # Minimum relative altitude that a controller can order


# Abstract object for wrapping a control-order-issuing ACS ROS object 
# that works by generating waypoint commands that can be contained in
# an already established node or run as its own independent node
# Instantiated objects will provide a waypoint publisher that publishes
# computed waypoints to the appropriate topic.
#
# Class member variables:
#   _wpPublisher: publisher object for publishing waypoint commands
#
# Inherited from Controller
#   controllerID: identifier (int) for this particular controller
#   statusPublisher: publisher object for controller status
#   statusStamp: timestamp of the last status message publication
#   sequence: number of waypoint sequences that have been ordered
#   is_ready: set to True when a waypoint sequence has been loaded
#   is_active: set to True is the waypoint sequencer is running
#
# Inherited from Nodeable:
#   nodeName:  Name of the node to start or node in which the object is
#   timer: ROS rate object that controls the timing loop
#   DBUG_PRINT: set true to force screen debug messages (default FALSE)
#   WARN_PRINT: set false to force screen warning messages (default FALSE) 
class WaypointController(controller.Controller):

    # Class initializer sets up the publisher for the waypoint topic
    # @param nodename:  name of the node that the object is contained in
    # @param ctrlrID: identifier (int) for this particular controller
    def __init__(self, nodename, ctrlrID):
        controller.Controller.__init__(self, nodename, ctrlrID)
        self._wpPublisher = self.createPublisher("payload_waypoint", LLA)


    #---------------------------------------------------------
    # Class-specific methods implementing class functionality
    #---------------------------------------------------------

    # Publishes a computed waypoint to the autopilot topic.  This method enforces
    # an altitude check to make sure the published waypoint is not below the minimum
    # safe altitude.  Additional safety checks can be implemented here as required.
    # @param wp: computed waypoint (LLA)
    def publishWaypoint(self, wp):
        if wp.alt >= MIN_REL_ALT: #verify valid altitude order
            self._wpPublisher.publish(wp)
            self.log_dbug("Sent to (%0.06f, %0.06f, %0.03f" \
                              %(wp.lat, wp.lon, wp.alt))
        else:
            self.set_active(False)
            self.log_warn("Altitude control mode invalid or invalid altitude order")

