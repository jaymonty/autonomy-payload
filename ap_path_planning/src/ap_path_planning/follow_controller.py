#!/usr/bin/env python

# Follow Me code implemented as a node.
# Listens to incoming network poses, downsamples, 
# does some math, and sends coords to local autopilot
# Improved version written by Stefan Jorgensen, August 2014
#
# Modified by Duane Davis (object implementation)

import sys
import rospy
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from argparse import ArgumentParser

# Import ROS message and service types
import std_msgs.msg as stdmsg

import math
import ap_lib.ap_enumerations as enums
from ap_lib import gps_utils
from ap_lib import nodeable
from ap_lib.waypoint_controller import *
from ap_lib.controller import *

# Base name for node topics and services
NODENAME = 'follower'

#Global variables (constants)
BASE_ALT_MODE = 0  # enumeration indicating base altitude mode
ALT_SEP_MODE = 1   # enumeration indicating altitude separation mode

OVERSHOOT = 250.0      # default "ahead" distance to place waypoint to avoid capture
FOLLOW_DISTANCE = 50.0 # default distance behind the lead to place the follow point
MAX_LOOKAHEAD = 10.0   # max DR time for intercept point computation

# Object that creates or receives waypoint sequences and monitors the
# vehicle's progress through the series of waypoints
#
# Class member variables:
#   ownID:  ID of this aircraft
#   ownLat: latitude of this aircraft
#   ownLon: longitude of this aircraft
#   ownAlt: altitude of this aircraft
#   ownRelAlt: altitude of this aircraft relative to launch altitude
#   ownVx: north/south linear velocity (m/s) of this aircraft
#   ownVy: east/west linear velocity (m/s) of this aircraft
#   ownBaseAlt: altitude from which relative altitude is measured
#   followID: ID of the leader aircraft (follow this one)
#   followLat: latitude of the aircraft being followed
#   followLon: longitude of the aircraft being followed
#   followAlt: altitude of the aircraft being followed
#   followRelAlt: altitude of the aircraft being followed relative to launch altitude
#   altMode: indicates whether base altitude or altitude separation is utilized
#   ctrlAlt: control altitude (base altitude or separation in meters)
#   rFollow: target following distance from the leader aircraft (meters)
#   rOffset: clockwise angle (radians) from direct astern for the follow point
#   tgtLat: computed target latitude for the follow position
#   tgtLon: computed target longitude for the follow position
#   tgtCrs: computed vehicle course at the follow position (matches leader course)
#   rOvershoot: overshoot distance for avoiding loiter behavior (meters)
#
# Inherited from WaypointController
#   _wpPublisher: publisher object for computed waypoints to be sent to the autopilot
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
#
# Class member functions:
#   callBackSetup: implementation of the Nodeable virtual function
#   runController: implementation of the Controller virtual function
#   reset: used to set follow control parameters
#   _compute_follow_wp: computes a target waypoint to achieve follow behavior
#   _process_formation_order: callback for messages to the formation order topic
#   _swarm_callback: callback for swarm state messages
class FollowController(WaypointController):

    # Class initializer initializes class variables.
    # This assumes that the object is already running within an initialized
    # ROS node (i.e., the object does not initialize itself as a node).
    # This enables multiple objects to run within a single node if
    # required.  The initializer does some parameter checking (mostly
    # whether or not all of the required parameters are there), but does
    # not check ranges, magnitudes, or signs.
    # @param nodename: name of the ROS node in which this object exists
    # @param ownAC: ID (int) of this aircraft
    def __init__(self, nodename, ownAC):
        WaypointController.__init__(self, nodename, enums.FOLLOW_CTRLR)
        self.ownID = ownAC
        self.ownLat = None
        self.ownLon = None
        self.ownAlt = None
        self.ownRelAlt = None
        self.ownVx = None
        self.ownVy = None
        self.baseAlt = None
        self.followID = None
        self.followLat = None
        self.followLon = None
        self.followAlt = None
        self.followRelAlt = None
        self.followVx = None
        self.followVy = None
        self.rFollow = None
        self.rOvershoot = None
        self.rOffset = None
        self.tgtLat = None
        self.tgtLon = None
        self.tgtCrs = None
        self.altMode = None
        self.ctrlAlt = None
        self.statusStamp = None
#        self.DBUG_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Establishes the callbacks for the FollowController object.  The object
    # subscribes to the swarm_tracker/swarm_uav_states topic for own-aircraft
    # and follow aircraft state updates
    # @param params: no additional parameters required by this method
    def callbackSetup(self, params=[]):
        self.createSubscriber("swarm_uav_states", apmsg.SwarmStateStamped, \
                              self._swarm_callback)
        self.createSubscriber("%s_set"%self.nodeName, \
                              apmsg.FormationOrderStamped, \
                              self._process_formation_order)


    # Executes one iteration of the controller for the FollowController object
    # The loop computes a target waypoint and publishes it to the
    # /autopilot/payload_waypoint ROS topic.
    def runController(self):
        if self.ownLat is None:
            return

        target_wp = self._compute_follow_wp()
        # Only send if there is valid data to use
        if target_wp:

            lla = apbrg.LLA()
            lla.lat = target_wp[0]
            lla.lon = target_wp[1]
            lla.alt = None

            # Set altitude of command (base alt or offset from leader altitude)
            # NOTE:  The autopilot expects issued orders to use RELATIVE altitude
            #        Don't use absolute (MSL) altitude in the LLA message!!!
            if self.altMode == BASE_ALT_MODE:
                lla.alt = self.ctrlAlt - self.baseAlt
            elif self.altMode == ALT_SEP_MODE:
                lla.alt = self.followAlt + self.ctrlAlt - self.baseAlt

            self.publishWaypoint(lla)


    # Pauses and unpauses the controller (will not have any effect on a
    # controller that is not active).  This method overrides the Controller
    # class method so that the waypoint position can be set to the current
    # UAV position.  The parent class method is explicitly called at the
    # end of this implementation.
    # @param pause: Boolean value to activate or pause or unpause
    def set_pause(self, pause):
        if not self.is_active: return
        if pause:
            lla = apbrg.LLA()
            lla.lat = self.ownLat
            lla.lon = self.ownLon
            lla.alt = self.ownRelAlt
            self.publishWaypoint(lla)

        return super(FollowController, self).set_pause(pause)


    #--------------------------
    # Object-specific functions
    #--------------------------

    # Resets the leader and follow parameters information
    # @param followAC:  ID of the aircraft to be followed
    # @param followAC: ID of the aircraft to be followed
    # @param followDist: distance behind the lead aircraft to aim for
    # @param offset: angle from astern (radians) for the point
    # @param altMode: altitude mode (BASE_ALT_MODE or ALT_SEP_MODE)
    # @param ctrlAlt: control altitude (follow altitude or vertical separation)
    def reset(self, followAC, followDist, offset, altMode, ctrlAlt):
        self.followLat = None
        self.followLon = None
        self.followAlt = None
        self.followRelAlt = None
        self.followVx = None
        self.followVy = None

        if (altMode != BASE_ALT_MODE) and (altMode != ALT_SEP_MODE):
            self.set_ready_state(False)
            self.log_warn("Must specify base altitude or altitude separation mode")
            return

        if (followAC == self.ownID):
            self.set_ready_state(False)
            self.log_warn("attempt to order aircraft to follow itself")
            return

        self.followID = followAC
        self.rFollow = followDist
        self.rOvershoot = OVERSHOOT
        self.rOffset = offset
        self.altMode = altMode
        self.ctrlAlt = ctrlAlt
        self._sequence += 1
        self.set_ready_state(True)
        self.log_dbug("formatio-n command: ldr=%d, range=%f, offset=%f"%(followAC, followDist, offset))


    # Compute a target waypoint to  with overshoot
    def _compute_follow_wp(self):
        # Need data for both own aircraft and leader aircraft
        if (self.ownLat is None) or (self.ownLon is None):
            self.log_dbug("cannot compute target waypoint without self data")
            return None

        if (self.followLat is None) or (self.followLon is None):
            self.log_dbug("cannot compute target waypoint without leader data")
            return None

        # Project back from leader to the follow point
        self.tgtCrs = math.atan2(self.followVy, self.followVx);
        self.tgtLat, self.tgtLon = \
            gps_utils.gps_newpos(self.followLat, self.followLon, \
                                 (self.tgtCrs + self.rOffset), self.rFollow)

        # Project fwd from follow point to compute a rough "intercept" point
        time_to_intercept = 0.0
        if (abs(self.ownVx) > 0.1) or (abs(self.ownVy) > 0.1):  # Avoid div by 0!
            time_to_intercept = gps_utils.gps_distance(self.ownLat, self.ownLon, \
                                                       self.tgtLat, self.tgtLon) / \
                                math.hypot(self.ownVx, self.ownVy)
        time_to_intercept = max(time_to_intercept, MAX_LOOKAHEAD)

        tgt_travel = math.hypot(self.followVx, self.followVy) * time_to_intercept
        self.tgtLat, self.tgtLon = \
            gps_utils.gps_newpos(self.tgtLat, self.tgtLon, self.tgtCrs, tgt_travel)

        # Choose a point 1 "overshoot" towards the target point
        to_follow_pt = gps_utils.gps_bearing(self.ownLat, self.ownLon, \
                                             self.tgtLat, self.tgtLon)
        self.tgtLat, self.tgtLon = \
            gps_utils.gps_newpos(self.ownLat, self.ownLon, to_follow_pt, self.rOvershoot)

        return (self.tgtLat, self.tgtLon)


    #------------------------------------------
    # ROS Subscriber callbacks -for this object
    #------------------------------------------

    # Process incoming formation order message
    # @param formMsg: message containing formation requirements (FormationOrderStamped)
    def _process_formation_order(self, formMsg):
        self.reset(formMsg.order.leader_id, formMsg.order.range, formMsg.order.angle, \
                   formMsg.order.alt_mode, formMsg.order.control_alt)


    # Handle incoming swarm_uav_states messages
    # @param swarmMsg: message containing swarm data (SwarmStateStamped)
    def _swarm_callback(self, swarmMsg):
        selfUpdated = False
        leaderUpdated = False
        for swarmAC in swarmMsg.swarm:
            if selfUpdated and leaderUpdated:
                break;  # Don't loop more than necessary

            if swarmAC.vehicle_id == self.ownID:
                self.ownLat = swarmAC.state.pose.pose.position.lat
                self.ownLon = swarmAC.state.pose.pose.position.lon
                self.ownAlt = swarmAC.state.pose.pose.position.alt
                self.ownRelAlt = swarmAC.state.pose.pose.position.rel_alt
                self.ownVx = swarmAC.state.twist.twist.linear.x
                self.ownVy = swarmAC.state.twist.twist.linear.y
                self.baseAlt = self.ownAlt - self.ownRelAlt
                selfUpdated = True

            elif swarmAC.vehicle_id == self.followID:
                if self.is_active and swarmAC.swarm_state == enums.AP_ERROR:
                    self.set_ready_state(False)
                    self.log_warn("leader UAV not in auto mode, disabling controller")

                self.followLat = swarmAC.state.pose.pose.position.lat
                self.followLon = swarmAC.state.pose.pose.position.lon
                self.followAlt = swarmAC.state.pose.pose.position.alt
                self.followRelAlt = swarmAC.state.pose.pose.position.rel_alt
                self.followVx = swarmAC.state.twist.twist.linear.x
                self.followVy = swarmAC.state.twist.twist.linear.y
                leaderUpdated = True

        if self.is_active and not leaderUpdated:
            self.set_ready_state(False)
            self.log_warn("leader aircraft not present in swarm update")

