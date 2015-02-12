#!/usr/bin/env python

# Object that manages swarm commands and determines how to set up controllers
# on this aircraft.
#
# Duane Davis 2014

# Standard python library imports
import sys
import math

# ROS library imports
import rospy
import std_msgs.msg as stdmsg

# ACS imports
import ap_msgs.msg as apmsg
import ap_srvs.srv as apsrv
import ap_lib.nodeable as nodeable
import ap_lib.controller as controller
import ap_path_planning.follow_controller as follower


# Object that ingests swarm commands and computes the appropriate controller
# settings for this particular aircraft.
# NOTE:  The SwarmManager is "stateless" and does not remember previous swarm
#        orders.  Calculations on the state of the swarm when an order is
#        received.  For this reason, it is not idempotent (i.e., repeated
#        issuing of the same order may not yield the same result each time).
#
# Class member variables:
#   _ownID: ID of this aircraft
#   _ownSubswarmID: ID of the subswarm to which this vehicle belongs
#   _rqd_control_mode: ID (int) of the controller implementing the swarm command
#   _last_control_mode: ID (int) of the most recent set_selector_mode order
#   _swarm_state: object containing the state of all swarm aircraft
#   _subswarm_state: object containing the state of all subswarm aircraft
#   _follow_publisher: ROS publisher to the follow controller set topic
#   _ctlr_select_srv_proxy: ROS proxy for the ctlr_selector set mode service
#
# Inherited from Nodeable:
#   nodeName:  Name of the node to start or node in which the object is
#   timer: ROS rate object that controls the timing loop
#   DBUG_PRINT: set true to force screen debug messages (default FALSE)
#   WARN_PRINT: set false to force screen warning messages (default FALSE)
#
# Class member functions:
#   callBackSetup: implementation of the Nodeable virtual function
#   publisherSetup: implementation of the Nodeable virtual function
#   serviceSetup: implementation of the Nodeable virtual function
#   _swarm_sort: utility for sorting swarm aircraft by an arbitrary criteria
#   _process_swarm_state: callback for swarm state messages
#   _process_subswarm_state: callback for subswarm state messages
#   _process_selector_status: callback for ctlr_selector/status messages
#   _process_swarm_control_run: implements the activate/deactivate ROS service
#   _process_swarm_formation_order: callback for swarm formation messages
class SwarmManager(nodeable.Nodeable):

    # Class initializer initializes class variables.
    # This assumes that the object is already running within an initialized
    # ROS node (i.e., the object does not initialize itself as a node).
    # This enables multiple objects to run within a single node if
    # required.  The initializer does some parameter checking (mostly
    # whether or not all of the required parameters are there), but does
    # not check ranges, magnitudes, or signs.
    # @param nodename: name of the ROS node in which this object exists
    def __init__(self, nodename):
        nodeable.Nodeable.__init__(self, nodename)
        self._ownID = rospy.get_param("aircraft_id")
        self._rqd_control_mode = controller.NO_PAYLOAD_CTRL
        self._last_control_mode = controller.NO_PAYLOAD_CTRL
        self._swarm_state = None
        self._subswarm_state = None
        self._follow_publisher = None
        self._ctlr_select_srv_proxy = \
            rospy.ServiceProxy("ctlr_selector/set_selector_mode", \
                               apsrv.SetInteger)
#        self.DBUG_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Establishes the callbacks for the SwarmManager object.  The object
    # subscribes to the swarm_state and subswarm_state topics for
    # own-aircraft, swarm, and subswarm states, to the swarm_control_run
    # topic to initiate the actual swarm control, and to the set_selector_mode
    # topic to keep track of which payload mode the vehicle is currently in.
    # Swarm orders currently available are read from the following topics:
    #   Swarm formation:  swarm_formation_set
    # @param params: list of required parameters (none are at present)
    def callbackSetup(self, params=[]):
        self.createSubscriber("swarm_state", apmsg.SwarmStateStamped, \
                              self._process_swarm_state)
        self.createSubscriber("subswarm_state", apmsg.SwarmStateStamped, \
                              self._process_subswarm_state)
        self.createSubscriber("selector_status", \
                              apmsg.ControllerGroupStateStamped, \
                              self._process_selector_status)
        self.createSubscriber("swarm_formation_set", \
                              apmsg.SwarmFormationOrderStamped, \
                              self._process_swarm_formation_order)


    # Establishes the publishers for the SwarmManager object.  The object
    # publishes to the set_selector_mode topic to initiate control as
    # computed by the SwarmManager.
    # @param params: list of required parameters (none are at present)
    def publisherSetup(self, params=[]):
        self._ctlr_select_publisher = \
            self.createPublisher("set_selector_mode", stdmsg.UInt8, 1)
        self._follow_publisher = \
            self.createPublisher("follower_set", apmsg.FormationOrderStamped, 1)


    # Establishes the services for the SwarmManager object.  The object
    # uses the swarm_control_run service to activate and deactivate
    # the required control mode.
    # @param params: list of required parameters (none are at present)
    def serviceSetup(self, params=[]):
        self.createService("swarm_control_run", \
                           apsrv.SetBoolean, self._process_swarm_control_run)


    #--------------------------
    # Object-specific functions
    #--------------------------

    # Sorts a list of tuples into ascending order.  The tuples are
    # assumed to be of the form [ acftID, sortCriteria ].  The list
    # itself is not affected (i.e., a new list is created)
    # @param swarm_pairs:  list of tuples to be sorted
    # @return a sorted list
    def _swarm_sort(self, swarm_pairs):
        return sorted(swarm_pairs, key = lambda tup: tup[1])


    #-----------------------------------------------------
    # ROS service implementation functions for this object
    #-----------------------------------------------------

    # Handle swarm control activation/deactivation service requests
    # @param actavateSrv: Activation/deactivation request (ap_srvs/SetBoolean)
    def _process_swarm_control_run(self, activateSrv):
        newMode = controller.NO_PAYLOAD_CTRL
        if activateSrv.enable:
            newMode = self._rqd_control_mode
        resp = self._ctlr_select_srv_proxy(newMode)
        if resp.result:
            self.log_dbug("swarm command activation: " + str(newMode) + " success")
        else:
            self.log_dbug("swarm command activation: " + str(newMode) + " failed")
        return apsrv.SetBooleanResponse(resp.result)


    #------------------------------------------
    # ROS Subscriber callbacks -for this object
    #------------------------------------------

    # Handle incoming swarm_state messages
    # @param swarmMsg: message containing swarm data (SwarmStateStamped)
    def _process_swarm_state(self, swarmMsg):
        self._swarm_state = swarmMsg


    # Handle incoming swarm_state messages
    # @param swarmMsg: message containing swarm data (SwarmStateStamped)
    def _process_subswarm_state(self, subswarmMsg):
        self._subswarm_state = subswarmMsg


    # Process ctlr_selector status messages 
    # (only rqd to keep track of the "active" controller)
    # @param statusMsg: selector mode message
    def _process_selector_status(self, statusMsg):
        self._last_control_mode = statusMsg.state.active_controller


    # Specific swarm command callbacks

    # Process incoming swarm formation order message.
    # Computes the required formation order for this aircraft and publishes it
    # to the follower_set topic.
    # NOTE:  This behavior operates at the subswarm level (i.e., if the swarm
    #        formation order applies to subswarm other than the one to which this
    #        vehicle belongs, it will be ignored.
    # @param formMsg: swarm formation requirements msg (SwarmFormationOrderStamped)
    def _process_swarm_formation_order(self, formMsg):
        swarmRecs = []
        appliesTo = self._swarm_state.swarm
        if formMsg.order.subswarm_only:
            appliesTo = self._subswarm_state.swarm

        # sort by altitude & find this acft in the list
        for acft in appliesTo:
            swarmRecs.append([ acft.vehicle_id, acft.state.pose.pose.position.alt ])
        swarmRecs = self._swarm_sort(swarmRecs)
        if formMsg.order.alt_separation < 0.0:   # Descending order for stepdown (lead-high)
            swarmRecs.reverse()
        ownIndex = 0
        while swarmRecs[ownIndex][0] != self._ownID:
            ownIndex += 1

        if ownIndex == 0:  # This is the "lead" aircraft
            self._rqd_control_mode = self._last_control_mode
            self.log_dbug("swarm formation set as lead aircraft")
        else:
            self._rqd_control_mode = controller.FOLLOW_CTRLR
            order = apmsg.FormationOrderStamped()
            order.header.stamp = rospy.Time.now()
            order.order.leader_id = swarmRecs[ownIndex - 1][0]
            order.order.range = formMsg.order.range
            order.order.angle = formMsg.order.angle
            order.order.control_alt = formMsg.order.alt_separation
            order.order.alt_mode = follower.ALT_SEP_MODE
            self._follow_publisher.publish(order)
            self.log_dbug("swarm formation set as dash " + str(ownIndex + 1))


#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    swarm_manager = SwarmManager("swarm_manager")
    swarm_manager.runAsNode(10.0, [], [], [])

