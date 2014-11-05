#!/usr/bin/env python

#-------------------------------------------------------------------------
# Nodeable
# Duane Davis, 2014
#
# Defines an abstract class for wrapping ACS controllers into ROS objects
# that can be incorporated into other classes or run as independent nodes
#-------------------------------------------------------------------------

# ROS imports
import rospy
from rospy import rostime
from ap_lib import nodeable
import ap_msgs.msg as apmsg


# Global variables (constants)
CTRLR_BASENAME = 'controllers' # Default base name for controller topics

# Global constants for "known" controller types (essentially an enumeration)
NO_PAYLOAD_CTRL = 0
WP_SEQUENCE_CTRLR = 1
FOLLOW_CTRLR = 2

# Abstract object for wrapping a control-order-issuing ACS ROS object 
# that can be contained in an already established node or run as its
# own independent node
#
# Class member variables:
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
class Controller(nodeable.Nodeable):

    # Class initializer initializes base class member variables
    # @param nodename:  name of the node that the object is contained in
    # @param ctrlrID: identifier (int) for this particular controller
    def __init__(self, nodename, ctrlrID):
        nodeable.Nodeable.__init__(self, nodename)
        self.controllerID = ctrlrID
        self.sequence = 0
        self.is_ready = False
        self.is_active = False
        self.statusPublisher = None
        self.statusInterval = None


    #---------------------------------------------------------
    # Parent class virtual functions implemented in subclasses
    #---------------------------------------------------------

    #---------------------------------------------------------
    # Class-specific methods implementing class functionality
    #---------------------------------------------------------

    # Publishes a ControllerState message to the 
    def sendStatusMessage(self):
        if self.statusStamp == None: self.statusStamp = rospy.Time.now()
        time = rospy.Time.now()
        interval = (time.secs + (time.nsecs / 1e9)) - \
                   (self.statusStamp.secs + (self.statusStamp.nsecs / 1e9))
        if interval >= 1.0:
            self.statusStamp = time
            status = apmsg.ControllerState()
            status.controller_id = self.controllerID
            status.sequence = self.sequence
            status.is_ready = self.is_ready
            status.is_active = self.is_active
            self.statusPublisher.publish(status)

