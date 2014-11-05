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
from ap_lib import gps_utils
from ap_lib import nodeable


# Base name for node topics and services
NODE_BASENAME = 'follow_controller'
TRKR_BASENAME = 'swarm_tracker'   # Default base name for swarm tracker topics
CTRLR_BASENAME = 'controllers'    # Default base name for controller topics
AP_BASENAME = 'autopilot'         # Default base name for autopilot topics

#Global variables (constants)
BASE_ALT_MODE = 0  # enumeration indicating base altitude mode
ALT_SEP_MODE = 1   # enumeration indicating altitude separation mode

OVERSHOOT = 100.0      # default "ahead" distance to place waypoint to avoid capture
FOLLOW_DISTANCE = 50.0 # default distance behind the lead to place the follow point


# Object that creates or receives waypoint sequences and monitors the
# vehicle's progress through the series of waypoints
#
# Class member variables:
#   ownID:  ID of this aircraft
#   ownLat: latitude of this aircraft
#   ownLon: longitude of this aircraft
#   ownAlt: altitude of this aircraft
#   ownRelAlt: altitude of this aircraft relative to launch altitude
#   followID: ID of the leader aircraft (follow this one)
#   followLat: latitude of the aircraft being followed
#   followLon: longitude of the aircraft being followed
#   followAlt: altitude of the aircraft being followed
#   followRelAlt: altitude of the aircraft being followed relative to launch altitude
#   altMode: indicates whether base altitude or altitude separation is utilized
#   ctrlAlt: control altitude (base altitude or separation in meters)
#   rFollow: target following distance from the leader aircraft (meters)
#   rOffset: clockwise angle (radians) from direct astern for the follow point
#   rOvershoot: overshoot distance for avoiding loiter behavior (meters)
#   swarmSubscriber: subscriber object for the swarm state message topic
#   wpPublisher: publisher object for computed waypoints to be sent to the autopilot
#   sequence: number of waypoint sequences that have been ordered
#   is_ready: set to True when a waypoint sequence has been loaded
#   is_active: set to True is the waypoint sequencer is running
class FollowController(nodeable.Nodeable):

    # Class initializer initializes class variables.
    # This assumes that the object is already running within an initialized
    # ROS node (i.e., the object does not initialize itself as a node).
    # This enables multiple objects to run within a single node if
    # required.  The initializer does some parameter checking (mostly
    # whether or not all of the required parameters are there), but does
    # not check ranges, magnitudes, or signs.
    # @param nodename: name of the ROS node in which this object exists
    # @param ownAC: ID (int) of this aircraft
    # @param followAC: ID of the aircraft to be followed
    # @param followDist: distance behind the lead aircraft to aim for
    # @param offset: clockwise angle from (radians) directly astern to aim for
    # @param overshoot: distance ahead of follow point to drive to (avoid capture)
    # @param altMode: altitude mode (BASE_ALT_MODE or ALT_SEP_MODE)
    # @param ctrlAlt: control altitude (follow altitude or vertical separation)
    def __init__(self, nodename, ownAC):
        nodeable.Nodeable.__init__(self, nodename)
        self.ownID = ownAC
        self.sequence = 0
        self.is_ready = False
        self.is_active = False
        self.ownLat = None
        self.ownLon = None
        self.ownAlt = None
        self.ownRelAlt = None
        self.swarmSubscriber = None
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
        self.altMode = None
        self.ctrlAlt = None
        self.DBUG_PRINT = True
        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Establishes the callbacks for the FollowController object.  The object
    # subscribes to the swarm_tracker/swarm_state topic for own-aircraft
    # and follow aircraft state updates
    # @param params: list as follows: [ swarm_tracker_base_name, controller_base_name ]
    def callbackSetup(self, params=[ TRKR_BASENAME, CTRLR_BASENAME ]):
        rospy.Subscriber("%s/swarm_state"%params[0], \
                         apmsg.SwarmStateStamped, self.swarm_callback)
        rospy.Subscriber("%s/follower_run"%params[1], 
                         stdmsg.Bool, self.process_run_message)
        rospy.Subscriber("%s/set_form_params"%params[1],
                         apmsg.FormationOrder, self.process_formation_order)


    # Sets up publishers for the FollowController object.  The object publishes
    # autopilot_bridge.LLA messages to the payload_waypoint topic
    # @param params: list as follows: [ autopilot_base_name ]
    def publisherSetup(self, params=[ AP_BASENAME]):
        self.wpPublisher = rospy.Publisher("%s/payload_waypoint"%params[0], apbrg.LLA)


    # Executes one iteration of the timed loop for the FollowController object
    # The loop computes a target waypoint and publishes it to the
    # /autopilot/payload_waypoint ROS topic.
    def executeTimedLoop(self):
        if not self.is_active or self.ownLat is None:
            return

        target_wp = self.compute_follow_wp()
        # Only send if there is valid data to use
        if target_wp:

            lla = apbrg.LLA()
            lla.lat = target_wp[0]
            lla.lon = target_wp[1]
            lla.alt = None

            # Set altitude of command (base alt or offset from leader altitude)
            if self.altMode == BASE_ALT_MODE:
                lla.alt = self.ctrlAlt
            elif self.altMode == ALT_SEP_MODE:
                lla.alt = self.followAlt + self.ctrlAlt

            if lla.alt is not None: #verify valid altitude data
                self.wpPublisher.publish(lla)
                self.log_dbug("Sent to (%0.06f, %0.06f, %0.03f (leader alt: %0.03f))" \
                              %(lla.lat, lla.lon, lla.alt, self.followAlt))
            else:
                self.log_warn("Altitude control mode invalid")


    #--------------------------
    # Object-specific functions
    #--------------------------

    # Resets the leader and follow parameters information
    # @param followAC:  ID of the aircraft to be followed
    # @param followAC: ID of the aircraft to be followed
    # @param followDist: distance behind the lead aircraft to aim for
    # @param offset: angle from astern (radians) for the point
    # @param overshoot: distance (meters) ahead of follow point to drive to (avoid capture)
    # @param altMode: altitude mode (BASE_ALT_MODE or ALT_SEP_MODE)
    # @param ctrlAlt: control altitude (follow altitude or vertical separation)
    def reset(self, followAC, followDist, offset, overshoot, altMode, ctrlAlt):
        self.followLat = None
        self.followLon = None
        self.followAlt = None
        self.followRelAlt = None
        self.followVx = None
        self.followVy = None

        if (altMode != BASE_ALT_MODE) and (altMode != ALT_SEP_MODE):
            self.is_active = False
            self.is_ready = False
            self.log_warn("Must specify base altitude or altitude separation mode")
            return

        if (followAC == self.ownID):
            self.is_active = False
            self.is_ready = False
            self.log_warn("attempt to order aircraft to follow itself")
            return

        self.followID = followAC
        self.rFollow = followDist
        self.rOvershoot = overshoot
        self.rOffset = offset
        self.altMode = altMode
        self.ctrlAlt = ctrlAlt
        self.is_ready = True
        self.sequence += 1
        self.log_dbug("formation command: ldr=%d, range=%f, offset=%f"%(followAC, followDist, offset))


    # Activates or deactivates the controller.  Will not activate if leader ID
    # is the same as the following aircraft (can't follow itself!)
    # @param activate: Boolean value to activate or deactivate the controller
    def set_active(self, activate):
        if not self.is_ready:
            self.is_active = False
            self.log_warn("attempt to activate uninitialized follow node")
        elif activate and self.followID == self.ownID:
            self.is_ready = False
            self.is_active = False
            self.log_warn("attempt to activate follower node to follow self")
        else:
            self.is_active = activate
            self.log_dbug("activation command: " + str(activate))


    # Compute a target waypoint to  with overshoot
    def compute_follow_wp(self):
        # Need data for both own aircraft and leader aircraft
        if (self.ownLat is None) or (self.ownLon is None):
            self.log_dbug("cannot compute target waypoint without self data")
            return None

        if (self.followLat is None) or (self.followLon is None):
            self.log_dbug("cannot compute target waypoint without leader data")
            return None

        # Project line back from leader as the follow target
        leader_course = math.atan2(self.followVy, self.followVx);
        trail_lat, trail_lon = \
            gps_utils.gps_newpos(self.followLat, self.followLon, \
                                 (leader_course - math.pi + self.rOffset), self.rFollow)

        # Project line forward from the follow point to avoid waypoint capture
        follow_course = gps_utils.gps_bearing(self.ownLat, self.ownLon, trail_lat, trail_lon)
        tgt_dist = gps_utils.gps_distance(self.ownLat, self.ownLon, trail_lat, trail_lon)
        return gps_utils.gps_newpos(self.ownLat, self.ownLon, follow_course, (tgt_dist + self.rOvershoot))


    #-----------------------------------------
    # ROS Subscriber callbacks for this object
    #-----------------------------------------

    # Handle incoming run message
    # @param startMsg: message activating or de-activating the controller (Boolean)
    def process_run_message(self, startMsg):
        self.set_active(startMsg.data)


    # Process incoming formation order message
    # @param formMsg: message containing formation requirements (FormationOrder)
    def process_formation_order(self, formMsg):
        self.reset(formMsg.leader_id, formMsg.range, formMsg.offset_angle, \
                   formMsg.overshoot, formMsg.alt_mode, formMsg.ctrl_alt)


    # Handle incoming swarm_state messages
    # @param swarmMsg: message containing swarm data (SwarmStateStamped)
    def swarm_callback(self, swarmMsg):
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
                selfUpdated = True

            elif swarmAC.vehicle_id == self.followID:
                self.followLat = swarmAC.state.pose.pose.position.lat
                self.followLon = swarmAC.state.pose.pose.position.lon
                self.followAlt = swarmAC.state.pose.pose.position.alt
                self.followRelAlt = swarmAC.state.pose.pose.position.rel_alt
                self.followVx = swarmAC.state.twist.twist.linear.x
                self.followVy = swarmAC.state.twist.twist.linear.y
                leaderUpdated = True
        if self.is_active and not leaderUpdated:
            self.is_ready = False
            self.is_active = False
            self.log_warn("leader aircraft not present in swarm update")

