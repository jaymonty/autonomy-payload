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
import autopilot_bridge.msg as apmsgs


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
#   _swarm_uav_states: dictionary object containing state of all swarm aircraft
#   _swarm_keys: list of all aircraft IDs (keys) in the swarm dictionary
#   _subswarm_keys: list of subswarm aircraft IDs (keys) in the swarm dictionary
#   _update_subswarm_publisher: ROS publisher to assign vehicle to a subswarm
#   _swarm_state_publisher: ROS publisher to publish the current swarming state
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
#   _process_subswarm_update: callback for update subswarm messages
#   _process_swarm_uav_states: callback for swarm state messages
#   _process_subswarm_uav_states: callback for subswarm state messages
#   _process_selector_status: callback for ctlr_selector/status messages
#   _process_swarm_control_run: implements the activate/deactivate ROS service
#   _process_swarm_formation_order: callback for swarm formation messages
class SwarmManager(nodeable.Nodeable):

    # Fixed (per convention) state-specific waypoints
    TAKEOFF_WP = 1       # Airborne
    SWARM_INGRESS_WP = 4 # Available for tasking
    SWARM_EGRESS_WP = 5  # Leaving swarm for recovery
    RACETRACK_WP = 7     # First racetrack waypoint

    # Enumeration for swarming states
    # Swarm Operator has control when IN_SWARM (subswarm active)
    # All other states under control of ground station or flight crew
    PRE_FLIGHT = 0   # Powered on, going through pre-fllight checks
    FLIGHT_READY = 1 # Awaiting launch
    LAUNCH = 2       # Airborne, waiting for handoff to swarm operator
    IN_SWARM = 3     # Swarming-available or swarming-active
    EGRESS = 4       # Transit to recovery staging (still required?)
    LANDING = 5      # Flight crew has control for landing
    ON_DECK = 6      # Aircraft has landed

    # For user interface use or debugging
    STATE_STRINGS = { PRE_FLIGHT: 'Preflight', \
                      FLIGHT_READY: 'Flight Ready', \
                      LAUNCH: 'Launched', \
                      IN_SWARM: 'Swarming', \
                      EGRESS: 'Egressing', \
                      LANDING: 'Landing', \
                      ON_DECK: 'On Deck' }


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
        self._subswarm_id = 0
        self._swarm_state = SwarmManager.PRE_FLIGHT
        self._rqd_control_mode = controller.NO_PAYLOAD_CTRL
        self._last_control_mode = controller.NO_PAYLOAD_CTRL
        self._swarm_uav_states = dict()
        self._swarm_keys = []
        self._subswarm_keys = []
#        self._subswarm_uav_states = None
        self._follow_publisher = None
        self._swarm_state_publisher = None
        self._ctlr_select_srv_proxy = \
            rospy.ServiceProxy("ctlr_selector/set_selector_mode", \
                               apsrv.SetInteger)
#        self.DBUG_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Establishes the callbacks for the SwarmManager object.  The object
    # subscribes to the swarm_uav_states and subswarm_uav_states topics for
    # own-aircraft, swarm, and subswarm states, to the swarm_control_run
    # topic to initiate the actual swarm control, and to the set_selector_mode
    # topic to keep track of which payload mode the vehicle is currently in.
    # Swarm orders currently available are read from the following topics:
    #   Swarm formation:  swarm_formation_set
    # @param params: list of required parameters (none are at present)
    def callbackSetup(self, params=[]):
        self.createSubscriber("swarm_uav_states", apmsg.SwarmStateStamped, \
                              self._process_swarm_uav_states)
#        self.createSubscriber("subswarm_uav_states", apmsg.SwarmStateStamped, \
#                              self._process_subswarm_uav_states)
        self.createSubscriber("selector_status", \
                              apmsg.ControllerGroupStateStamped, \
                              self._process_selector_status)
        self.createSubscriber("swarm_formation_set", \
                              apmsg.SwarmFormationOrderStamped, \
                              self._process_swarm_formation_order)
        self.createSubscriber("recv_subswarm", stdmsg.UInt8, \
                              self._process_subswarm_update)
        self.createSubscriber("status", apmsgs.Status, \
                              self._process_autopilot_status)


    # Establishes the publishers for the SwarmManager object.  The object
    # publishes to the set_selector_mode topic to initiate control as
    # computed by the SwarmManager.
    # @param params: list of required parameters (none are at present)
    def publisherSetup(self, params=[]):
        self._update_subswarm_publisher = \
            self.createPublisher("update_subswarm", stdmsg.UInt8, 1, True)
        self._follow_publisher = \
            self.createPublisher("follower_set", apmsg.FormationOrderStamped, 1)
        self._swarm_state_publisher = \
            self.createPublisher("swarm_state", stdmsg.UInt8, 1, True)

        # Publish one message now to initialize the latched publishers with "0"
        self._update_subswarm_publisher.publish(stdmsg.UInt8(0))
        self._swarm_state_publisher.publish(stdmsg.UInt8(SwarmManager.PRE_FLIGHT))


    # Establishes the services for the SwarmManager object.  The object
    # uses the swarm_control_run service to activate and deactivate
    # the required control mode.
    # @param params: list of required parameters (none are at present)
    def serviceSetup(self, params=[]):
        self.createService("swarm_control_run", \
                           apsrv.SetBoolean, self._process_swarm_control_run)


    # Executes one iteration of the timed loop.  At present, with the exception
    # of testing for a transition from the PRE_FLIGHT to the FLIGHT_READY state
    # the method does not do anything (functionality can be added later asreq).
    def executeTimedLoop(self):
        if ((self._swarm_state == SwarmManager.PRE_FLIGHT) and \
            (rospy.has_param("flight_ready")) and \
            (bool(rospy.get_param("flight_ready")))):
                self._swarm_state = SwarmManager.FLIGHT_READY
                self._swarm_state_publisher.publish(stdmsg.UInt8(SwarmManager.FLIGHT_READY))
                self.log_dbug("Swarm state updated to %d" %self._swarm_state)


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

    # Handle incoming swarm_uav_states messages
    # @param swarmMsg: message containing swarm data (SwarmStateStamped)
    def _process_swarm_uav_states(self, swarmMsg):
        self._swarm_uav_states.clear()
        del self._swarm_keys[:]
        del self._subswarm_keys[:]
#        self._swarm_uav_states = swarmMsg
        for vehicle in swarmMsg.swarm:
            self._swarm_uav_states[vehicle.vehicle_id] = vehicle
            self._swarm_keys.append(vehicle.vehicle_id)
            if vehicle.subswarm_id == self._subswarm_id:
                self._subswarm_keys.append(vehicle.vehicle_id)


    # Handle incoming swarm_uav_states messages
    # @param swarmMsg: message containing swarm data (SwarmStateStamped)
#    def _process_subswarm_uav_states(self, subswarmMsg):
#        self._subswarm_uav_states = subswarmMsg


    # Process ctlr_selector status messages 
    # (only rqd to keep track of the "active" controller)
    # @param statusMsg: selector mode message
    def _process_selector_status(self, statusMsg):
        self._last_control_mode = statusMsg.state.active_controller


    # Process recv_subswarm messages.  Messages will be ignored if the
    # vehicle's swarm_state is anything other than IN_SWARM, in which case
    # the new swarm assignment will be published to the update_swwarm topic
    # @param subswarmMsg: message containing new subswarm assignment
    def _process_subswarm_update(self, subswarmMsg):
        if self._swarm_state == SwarmManager.IN_SWARM:
            self._subswarm_id = subswarmMsg.data
            self._update_subswarm_publisher.publish(stdmsg.UInt8(subswarmMsg.data))
            self.log_dbug("Subswarm updated to %d" %self._subswarm_id)
        else:
            self.log_warn("Cannot assign to subswarm %d in swarm state %s" \
                          %(subswarmMsg.data, SwarmManager.STATE_STRINGS[self._swarm_state]))


    # Process acs_pose messages.  Messages are tested primarily to identify
    # vehicle launch (i.e., current state == PRE_FLIGHT and aircraft is now
    # airborne) and landed (i.e., aircraft was airborne, but is now on deck)
    # NOTE:  Not required in SITL (callback disabled)--verify in aircraft
    # @param poseMsg: Geodometry message with the new pose
    def _process_acs_pose(self, poseMsg):
        if ((self._swarm_state == SwarmManager.PRE_FLIGHT) or \
            (self._swarm_state == SwarmManager.FLIGHT_READY)):
            vel = math.hypot(poseMsg.twist.twist.linear.x, \
                             poseMsg.twist.twist.linear.y);
            # If it's moving and above ground, it's flying
            if ((vel >= 5.0) and (poseMsg.pose.pose.position.rel_alt > 5.0)):
                self._swarm_state = SwarmManager.LAUNCH
                self._swarm_state_publisher.publish(stdmsg.UInt8(SwarmManager.LAUNCH))
                self.log_dbug("Swarm state updated to %d" %self._swarm_state)


    # Process autopilot status messages.  Messages are tested primarily to
    # detect swarm_state changes as follows:
    #    PRE_FLIGHT or FLIGHT_READY to LAUNCH when mis_cur goes fm 0 to 1
    #    LAUNCH to IN_SWARM when mis_cur goes to SWARM_INGRESS_WP or higher
    #    LAUNCH or IN_SWARM to EGRESS when mis_cur goes to SWARM_EGRESS_WP
    #    Anything to LANDING when mode goes to RTL
    #    LANDING to ON_DECK when as_read <= 5.0
    # @param statusMsg: Status message
    def _process_autopilot_status(self, statusMsg):
        if ((self._swarm_state == SwarmManager.PRE_FLIGHT) or \
            (self._swarm_state == SwarmManager.FLIGHT_READY)):
            if (statusMsg.mis_cur == SwarmManager.TAKEOFF_WP):
                self._swarm_state = SwarmManager.LAUNCH
                self._swarm_state_publisher.publish(stdmsg.UInt8(SwarmManager.LAUNCH))
                self.log_dbug("Swarm state updated to %d" %self._swarm_state)

        elif (self._swarm_state == SwarmManager.LAUNCH):
            if (statusMsg.mis_cur >= SwarmManager.SWARM_INGRESS_WP):
                self._swarm_state = SwarmManager.IN_SWARM
                self._swarm_state_publisher.publish(stdmsg.UInt8(SwarmManager.IN_SWARM))
                self.log_dbug("Swarm state updated to %d" %self._swarm_state)
            elif (statusMsg.mis_cur == SwarmManager.SWARM_EGRESS_WP):
                self._swarm_state = SwarmManager.EGRESS
                self._swarm_state_publisher.publish(stdmsg.UInt8(SwarmManager.EGRESS))
                self.log_dbug("Swarm state updated to %d" %self._swarm_state)

        elif (self._swarm_state == SwarmManager.IN_SWARM):
            if (statusMsg.mis_cur == SwarmManager.SWARM_EGRESS_WP):
                self._swarm_state = SwarmManager.EGRESS
                self._swarm_state_publisher.publish(stdmsg.UInt8(SwarmManager.EGRESS))
                self.log_dbug("Swarm state updated to %d" %self._swarm_state)

        if (self._swarm_state == SwarmManager.LANDING):
            if (statusMsg.as_read <= 5.0):
                self._swarm_state = SwarmManager.ON_DECK
                self._swarm_state_publisher.publish(stdmsg.UInt8(SwarmManager.ON_DECK))
                self.log_dbug("Swarm state updated to %d" %self._swarm_state)

        # Can transition to LANDING at any time after launch
        elif ((self._swarm_state != SwarmManager.PRE_FLIGHT) and \
            (self._swarm_state != SwarmManager.FLIGHT_READY) and \
            (statusMsg.mode == apmsgs.Status.MODE_RALLY)):
                self._swarm_state = SwarmManager.LANDING
                self._swarm_state_publisher.publish(stdmsg.UInt8(SwarmManager.LANDING))
                self._subswarm_id = 0
                self._update_subswarm_publisher.publish(stdmsg.UInt8(self._subswarm_id))
                self.log_dbug("Swarm state updated to %d" %self._swarm_state)
              

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
        appliesTo = self._swarm_uav_states.swarm
        if formMsg.order.subswarm_only:
            appliesTo = self._subswarm_uav_states.swarm

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

