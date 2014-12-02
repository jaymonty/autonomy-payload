#!/usr/bin/env python

#-----------------------------------------------------------------------
# Multiple waypoint sequencer
# Duane Davis, 2014
#
# Generates sequences of waaypoints and issues them to the autopilot in
# order based on the waypoint sequence and vehicle location
#-----------------------------------------------------------------------

#----------------------------
# Import a bunch of libraries
#----------------------------

# Standard Python imports
from collections import deque
import math

# General ROS imports
import rospy


# Import ROS message and service types
import std_msgs.msg as stdmsg

# Project-specific imports
from ap_lib.nodeable import *
from ap_lib.controller import *
from ap_lib.gps_utils import *
import ap_msgs.msg as apmsg
import autopilot_bridge.msg as apbrg


# Base name for node topics and services
NODE_BASENAME = 'wp_sequencer'
AP_BASENAME = 'autopilot'


# Other global constants
CAPTURE_DISTANCE = 110.0


# Object that creates or receives waypoint sequences and monitors the
# vehicle's progress through the series of waypoints.  Individual 
# waypoints are specified using LLA message object containing latitude,
# longitude, and altitude (MSL) 
#
# Class member variables:
#   wpList: list (queue) of waypoints being followed
#   baseAlt: altitude (MSL) from which relative altitudes are computed
#   currentWP: waypoint to which the vehicle is currently transiting
#   pose: current vehicle position
#   wpPublisher: publisher object for publishing waypoint commands
#   captureDistance: distance from waypoint considered captured
#   listComplete: set to True when the last waypoint is reached
#   readyNextWP: set to True when the next waypoint can be issued
#
# Inherited from Controller:
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
class WaypointSequencer(Controller):

    # Class initializer initializes variables, subscribes to required
    # ROS topics, and creates required ros publishers.  Initializer
    # assumes that the object is already running within an initialized
    # ROS node (i.e., the object does not initialize itself as a node).
    # This enables multiple objects to run within a single node if
    # required.
    # @param nodeName: name of the ROS node for this object
    # @param waypoints: list of waypoints in the desired traversal order
    # @param apBaseName: base name of the ROS topic to publish waypoints to
    # @param captureDistance: distance from a waypoint considered good enough
    def __init__(self, nodeName=NODE_BASENAME, waypoints=[], \
                 captureDistance=CAPTURE_DISTANCE):
        Controller.__init__(self, nodeName, WP_SEQUENCE_CTRLR)
        self.currentWP = apbrg.LLA()
        self.pose = None
        self.baseAlt = 0.0
        self.setSequence(waypoints)
        self.captureDistance = captureDistance
        self.statusStamp = None
#        self.DBUG_PRINT = True
#        self.WARN_PRINT = True



    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Establishes the callbacks for the WaypointSequencer object.  The object
    # object subscribes to the acs_pose topic for current position
    # updates, the wp_sequencer_run topic for start and stop commands, and
    # the wp_list topic to receive new waypoint sequence lists
    # @param params: list as follows: [ odometry_base_name, controller_base_name ]
    def callbackSetup(self, params=[ AP_BASENAME, CTRLR_BASENAME ]):
        rospy.Subscriber("%s/acs_pose"%params[0], apbrg.Geodometry, \
                         self._updatePose)
        rospy.Subscriber("%s/wp_list"%params[1], apmsg.WaypointListStamped, \
                         self._receiveWaypointList)


    # Establishes the publishers for the WaypointSequencer object.  The object
    # publishes waypoint commands (LLA message) to the payload_waypoint topic
    # @param params: list as follows: [ autopilot_base_name, controller_base_name ]
    def publisherSetup(self, params=[ AP_BASENAME, CTRLR_BASENAME ]):
        self.wpPublisher = rospy.Publisher("%s/payload_waypoint"%params[0], apbrg.LLA)
        self.statusPublisher = rospy.Publisher("%s/status"%params[1], apmsg.ControllerState)


    # Executes one iteration of the timed loop for the WaypointSequencer
    # object. When "on", checks to see if the current waypoint has been
    # reached, and if so, issues the next one.
    def runController(self):
        self._checkReadyNextWP()
        if self.readyNextWP: self._incrementWP()


    #--------------------------
    # Object-specific functions
    #--------------------------

    # Updates the waypoint sequence.  Any existing waypoint sequence
    # will be deleted, and the new sequence will be immediately 
    # commenced.  If the new sequence contains no waypoints, any
    # existing sequence will be deleted, but any current sequenced
    # waypoint that was previously commanded will remain in effect
    # newWaypoints: list of waypoints (LLA) in the desired traversal order
    def setSequence(self, newWaypoints):
        try:
            self.wpList = deque(newWaypoints)
            if len(self.wpList) > 0:
                self.sequence += 1
                self.readyNextWP = True
                self.listComplete = False
                self.set_ready_state(True)
            else:
                self.readyNextWP = False
                self.listComplete = True
                self.set_ready_state(False)
            self.log_dbug("set waypoint sequence: " + str(newWaypoints))
        except Exception as exc:
            self.log_warn("exception resetting waypoint sequence: " + \
                          str(newWaypoints))
            self.wpList = deque()
            self.listComplete = True  # If there's been an error--quit!
            self.set_ready_state(False)


    # Adds a single waypoint to the end of the current list
    # @param newWaypoint:  the waypoint (LLA) to be added to the list
    def addWaypoint(self, newWaypoint):
        self.wpList.append(newWaypoint)
        self.listComplete = False
        self.set_ready_state(True)


    # Pops the first sequence from the waypoint list and updates
    # the currently commanded waypoint with the new value
    def _incrementWP(self):
        self.readyNextWP = False
        if (len(self.wpList) > 0):
            self.currentWP = self.wpList.popleft()
            self.currentWP.alt = self.currentWP.alt - self.baseAlt # convert to rel_alt

            # Make sure the wp is OK, and if so issue--if not kill everything!
            if self.currentWP.alt >= MIN_REL_ALT:
                self.wpPublisher.publish(self.currentWP)
                self.log_dbug("issuing new wpt: lat=" + str(self.currentWP.lat) +\
                                             ", lon=" + str(self.currentWP.lon) +\
                                             ", alt=" + str(self.currentWP.alt))
            else:
                self.set_ready_state(False)
                self.log_warn("ordered waypoint with negative altitude")
        else:
            self.log_dbug("reached last waypoint in sequence")
            self.listComplete = True
            self.set_ready_state(False)


    # Checks to see if it is time to issue the next waypoint
    def _checkReadyNextWP(self):
        try:
            if self.listComplete:  return False
            if self.readyNextWP:  return True
            d = gps_distance(self.pose[0], self.pose[1], \
                             self.currentWP.lat, self.currentWP.lon)
            if d < self.captureDistance:
                self.readyNextWP = True
                self.log_dbug("waypoint reached: lat=" + str(self.currentWP.lat) +\
                                              ", lon=" + str(self.currentWP.lon) +\
                                              ", alt=" + str(self.currentWP.alt))
            return self.readyNextWP
        except Exception as exc:
            self.log_warn("exception checking waypoint distance")
            print exc
            self.readyNextWP = False
            self.listComplete = True
            self.set_ready_state(False)
            return False


    #-----------------------------------------
    # ROS Subscriber callbacks for this object
    #-----------------------------------------

    # Handles new pose information for this aircraft so that the vehicle
    # can determine where it is relative to the current waypoint
    # @param poseMsg: Geodometry object with the new pose
    def _updatePose(self, poseMsg):
        self.pose = [ poseMsg.pose.pose.position.lat, \
                      poseMsg.pose.pose.position.lon ]
        self.baseAlt = poseMsg.pose.pose.position.alt - \
                       poseMsg.pose.pose.position.rel_alt


    # Handles waypoint lists written to the ROS topic
    # @param wptMsg: ROS message (WaypointListStamped) with the waypoint list
    def _receiveWaypointList(self, wpMsg):
        self.setSequence(spMsg.waypoints)

