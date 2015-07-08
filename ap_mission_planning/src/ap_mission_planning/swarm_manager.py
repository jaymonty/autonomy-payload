#!/usr/bin/env python

# Object that manages swarm commands and determines how to set up controllers
# on this aircraft.
#
# Duane Davis 2014

# Standard python library imports
import sys
import math
import time

# ROS library imports
import rospy
import std_msgs.msg as stdmsg

# ACS imports
import ap_msgs.msg as apmsg
import ap_srvs.srv as apsrv
import ap_lib.nodeable as nodeable
import ap_lib.controller as controller
import ap_lib.ap_enumerations as enums
import ap_path_planning.follow_controller as follower
import autopilot_bridge.msg as apmsgs
import autopilot_bridge.srv as apmsrv


# Object that ingests swarm commands and computes the appropriate controller
# settings for this particular aircraft.
# NOTE:  The SwarmManager is "stateless" and does not remember previous swarm
#        orders.  Calculations on the state of the swarm when an order is
#        received.  For this reason, it is not idempotent (i.e., repeated
#        issuing of the same order may not yield the same result each time).
#
# Class member variables:
#   _ownID: ID of this aircraft
#   _subswarm_id: ID of the subswarm to which this vehicle belongs
#   _behavior_activators: dictionary of swarm behavior activation functions
#   _egress_behavior_activators: dictionary of post-egress swarm behavior activation functions
#   _active_behavior: currently active swarm behavior
#   _swarm_uav_states: dictionary object containing state of all swarm aircraft
#   _swarm_keys: list of all aircraft IDs (keys) in the swarm dictionary
#   _subswarm_keys: list of subswarm aircraft IDs (keys) in the swarm dictionary
#   _waypoint_types: dictionary mapping waypoint IDs to waypoint type
#   _waypoint_alts: dictionary mapping waypoint IDs to altitudes (MSL)
#   _crnt_wpt_alt: most recently ordered waypoint altitude (MSL)
#   _update_subswarm_publisher: ROS publisher to assign vehicle to a subswarm
#   _swarm_state_publisher: ROS publisher to publish the current swarming state
#   _swarm_behavior_publisher: ROS publisher to publish the active swarm behavior
#   _follow_publisher: ROS publisher to the follow controller set topic
#   _wp_goto_publisher: ROS publisher to direct the vehicle to a waypoint #
#   _ctlr_select_srv_proxy: ROS proxy for the ctlr_selector set mode service
#   _wp_getrange_srv_proxy: ROS proxy for the wp_getrange service
#   _init_landing_sequencer_srv_proxy: ROS proxy for the init_landing_sequencer service
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
#   register_swarm_behavior: registers functions to implement swarm behaviors
#   _swarm_sort: utility for sorting swarm aircraft by an arbitrary criteria
#   _process_subswarm_update: callback for update subswarm messages
#   _process_swarm_uav_states: callback for swarm state messages
#   _process_subswarm_uav_states: callback for subswarm state messages
#   _process_selector_status: callback for ctlr_selector/status messages
#   _process_set_swarm_behavior: implements the set_swarm_behavior service
#   _set_swarm_state: method for updating the swarm state to a new value
#   _set_subswarm: method for normal or forced subswarm_id set
#   _get_subswarm_uavs: get a list of records of uavs in a particular subswarm
#   _activate_swarm_standby: implements the swarm standby behavior
#   _activate_egress: implements the egress behavior
#   _activate_fixed_follow: implements the fixed-following behavior
class SwarmManager(nodeable.Nodeable):

    FIXED_FOLLOW_DIST = 25.0  # Follower distance for "fixed follow" control
    TIMING_DELAY = 2.0        # Delay time to allow things to "take"
    LAUNCH_ALT = 20000.0      # AGL (m * 1000) altitude used to detect "launch"


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
        self._swarm_state = enums.PRE_FLIGHT
        self._active_behavior = enums.SWARM_STANDBY
        self._behavior_activators = dict()
        self._egress_behavior_activators = dict()
        self._swarm_uav_states = dict()
        self._waypoint_types = dict()
        self._waypoint_alts = dict()
        self._crnt_wpt_alt = 0.0
        self._swarm_keys = []
        self._subswarm_keys = []
        self._follow_publisher = None
        self._swarm_state_publisher = None
        self._ctlr_select_srv_proxy = None
        self._wp_getrange_srv_proxy = None
        self._init_landing_sequencer_srv_proxy = None
        self._init_takeoff_sequencer_srv_proxy = None
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
        self._swarm_behavior_publisher = \
            self.createPublisher("swarm_behavior", stdmsg.UInt8, 1, True)
        self._swarm_search_publisher = \
            self.createPublisher("swarmSearch_setup", apmsg.SwarmSearchOrderStamped, 1)
        self._wp_goto_publisher = \
            self.createPublisher("waypoint_goto", stdmsg.UInt16, 1, False)

        # Publish one message now to initialize the latched publishers with "0"
        self._set_swarm_behavior(enums.SWARM_STANDBY)
        self._set_swarm_state(enums.PRE_FLIGHT)
        self._set_subswarm(0)


    # Establishes the services for the SwarmManager object.  The object
    # uses the swarm_control_run service to activate and deactivate
    # the required control mode.
    # @param params: list of required parameters (none are at present)
    def serviceSetup(self, params=[]):
        self.createService("set_swarm_behavior", apsrv.SetInteger, \
                           self._process_set_swarm_behavior)
        self.createService("set_swarm_state", apsrv.SetInteger, \
                           self._process_set_swarm_state)
        self.createService("run_swarm_sequence_land", apsrv.SetInteger, \
                           self._process_set_sequence_land)
        self.createService("run_swarm_fixed_formation", apsrv.SetSwarmFormation, \
                           self._process_set_fixed_formation)
        self.createService("run_swarm_search", apsrv.SetSwarmSearchRequest, \
                           self._process_set_swarm_search)

    # Establishes the service proxies for the SwarmManager object.
    # @param params: list of required parameters (none are at present)
    def serviceProxySetup(self, params=[]):
        self._ctlr_select_srv_proxy = \
            self.createServiceProxy("set_selector_mode", apsrv.SetInteger)
        self._wp_getrange_srv_proxy = \
            self.createServiceProxy("wp_getrange", apmsrv.WPGetRange)
        self._init_landing_sequencer_srv_proxy = \
            self.createServiceProxy("init_landing_sequencer", apsrv.SetInteger)
        self._init_takeoff_sequencer_srv_proxy = \
            self.createServiceProxy("init_takeoff_sequencer", apsrv.SetInteger)


    # Executes one iteration of the timed loop.  At present, with the exception
    # of testing for a transition from the PRE_FLIGHT to the FLIGHT_READY state
    # the method does not do anything (functionality can be added later asreq).
    def executeTimedLoop(self):
        if ((self._swarm_state == enums.PRE_FLIGHT) and \
            (rospy.has_param("flight_ready")) and \
            (bool(rospy.get_param("flight_ready")))):
                self._set_swarm_state(enums.FLIGHT_READY)


    #--------------------------
    # Object-specific functions
    #--------------------------

    # Sets a new swarm state for the aircraft
    # @param newState:  new swarm state (int)
    def _set_swarm_state(self, newState):
        self._swarm_state = newState
        self._swarm_state_publisher.publish(stdmsg.UInt8(newState))
        self.log_dbug("Swarm state updated to %d" %self._swarm_state)


    # Sets a new subswarm for the aircraft
    # Can be used for either a normal or forced update
    # @param newSubswarm:  new subswarm ID (int)
    def _set_subswarm(self, newSubswarm):
        self._subswarm_id = newSubswarm
        self._update_subswarm_publisher.publish(stdmsg.UInt8(newSubswarm))
        self.log_dbug("Subswarm updated to %d" %self._subswarm_id)


    # Sets a new swarm behavior value for the aircraft
    # @param newBehavior:  updated subswarm behavior (int)
    def _set_swarm_behavior(self, newBehavior):
        self._active_behavior = newBehavior
        self._swarm_behavior_publisher.publish(stdmsg.UInt8(newBehavior))
        self.log_dbug("Active swarm behavior updated to %d" %self._active_behavior)


    # Used to register implementing functions for swarm behaviors.  The
    # function should take the service request object as a parameter so
    # that any required parameters (which will eventually be incorporated
    # into the message) will be available to the implementing function.
    # Implementing functions should return a Boolean value indicating
    # success or failure of the behavior initiation.
    # @param behavior_id: integer identifier of the behavior being registered
    # @param function: implementing function for the behavior
    # @param post_egress: true if the behavior is a post-egress behavior
    def register_swarm_behavior(self, behavior_id, function, post_egress = False):
        if not post_egress:
            self._behavior_activators[behavior_id] = function
        else:
            self._egress_behavior_activators[behavior_id] = function


    # Activates the swarm standby behavior (no payload control, vehicle sent
    # to the loiter point)
    # @param srvReq service request message (not used for this behavior)
    # @return Boolean value indicating behavior initiation success or failure
    def _activate_swarm_standby(self, srvReq=None):
        success = self._ctlr_select_srv_proxy(enums.NO_PAYLOAD_CTRL).result
        if success:
            time.sleep(SwarmManager.TIMING_DELAY) # give the controller switch time to take
            self._wp_goto_publisher.publish(stdmsg.UInt16(enums.SWARM_STANDBY_WP))
        return success


    # Activates the egress behavior
    # Note:  For now, this is hard coded to direct the vehicle to waypoint 5
    #        (the egress waypoint), but we will want to make this "smarter"
    #        eventually so that it doesn't depend on mission file organization
    # @param srvReq service request message (not used for this behavior)
    # @return Boolean value indicating behavior initiation success or failure
    def _activate_egress(self, srvReq=None):
        success = self._ctlr_select_srv_proxy(enums.NO_PAYLOAD_CTRL).result
        if success:
            time.sleep(SwarmManager.TIMING_DELAY) # give the controller switch time to take
            self._wp_goto_publisher.publish(stdmsg.UInt16(enums.SWARM_EGRESS_WP))
            self._set_swarm_state(enums.EGRESS)
        return success


    # Computes and activates the fixed-follow swarm behavior.  The most
    # recently observed altitudes deterine the follow order (high aircraft
    # leads).  If the aircraft is the lead aircraft, it is ordered to a
    # waypoint on the racetrack.  Otherwise the follower behavior is activated
    # @param srvReq service request message (not used for this behavior)
    # @return Boolean value indicating behavior initiation success or failure
    def _activate_fixed_follow(self, srvReq=None):
        success = False
        try:
            # Sort the participating aircraft by altitude
            low_to_high = []
            for uav in self._subswarm_keys:
                if uav == self._ownID:
                    low_to_high.append([ self._ownID, self._crnt_wpt_alt ])
                else:
                    low_to_high.append([ self._swarm_uav_states[uav].vehicle_id, \
                                         self._swarm_uav_states[uav].state.pose.pose.position.alt])
            low_to_high = sorted(low_to_high, key = lambda tup: tup[1])

            # ID the UAV to follow (own ID means this UAV is form lead)
            lead_ac_id = low_to_high[-1][0]
            if type(srvReq) == apsrv.SetSwarmFormationRequest and \
               not srvReq.stack_formation and lead_ac_id != self._ownID:
                for uav in range(len(low_to_high)-2, -1, -1):
                    if low_to_high[uav][0] == self._ownID:
                        lead_ac_id = low_to_high[uav+1][0]
                        break

            # If leader acft, set NO_PAYLOAD_CTRL and send to the racetrack
            if lead_ac_id == self._ownID:  # This AC is leader
                success = self._ctlr_select_srv_proxy(enums.NO_PAYLOAD_CTRL).result
                if success:
                    time.sleep(SwarmManager.TIMING_DELAY) # give the controller switch time to take
                    self._wp_goto_publisher.publish(stdmsg.UInt16(enums.RACETRACK_WP))

            else:
                distance = SwarmManager.FIXED_FOLLOW_DIST
                angle = math.pi
                if type(srvReq) == apsrv.SetSwarmFormationRequest:
                    distance = srvReq.distance
                    angle = srvReq.angle
                form_msg = apmsg.FormationOrderStamped()
                form_msg.header.stamp = rospy.Time.now()
                form_msg.header.frame_id = 'base_footprint'
                form_msg.order.leader_id = lead_ac_id
                form_msg.order.range = distance
                form_msg.order.angle = angle
                form_msg.order.alt_mode = follower.BASE_ALT_MODE
                form_msg.order.control_alt = self._crnt_wpt_alt
                self._follow_publisher.publish(form_msg)
                time.sleep(SwarmManager.TIMING_DELAY) # give the form set message a second to take
                success = self._ctlr_select_srv_proxy(enums.FOLLOW_CTRLR).result

        except Exception as ex:
            # Should never get here, but just in case
            self.log_warn("Set Control Mode: " + str(ex))

        return success


    # Activates the swarm sequenced landing behavior.
    # @param srvReq service request message (not used for this behavior)
    # @return Boolean value indicating behavior initiation success or failure
    def _activate_sequence_land(self, srvReq):
        success = False
        try:
            # Initialize the controller
            init = self._init_landing_sequencer_srv_proxy(srvReq.setting).result
            if not init:
                self.log_warn("Failed to initialize Swarm Landing Sequencer")
                raise Exception("Failed to initialize Swarm Landing Sequencer")

            success = self._ctlr_select_srv_proxy(enums.LANDING_SEQUENCE_CTRLR).result

        except Exception as ex:
            self.log_warn("Landing sequencer start exception: " + str(ex))

        return success

    # Activates the swarm search behavior.
    # @param srvReq service request message (not used for this behavior)
    # @return Boolean value indicating behavior initiation success or failure
    def _activate_swarm_search(self, searchReq=None):
        success = False
        try:
            swarmSearch_msg = apmsg.SwarmSearchOrderStamped()
            swarmSearch_msg.header.stamp = rospy.Time.now()
            swarmSearch_msg.order.searchAreaLength = searchReq.searchAreaLength
            swarmSearch_msg.order.searchAreaWidth = searchReq.searchAreaWidth
            swarmSearch_msg.order.lat = searchReq.lat
            swarmSearch_msg.order.lon = searchReq.lon
            swarmSearch_msg.order.masterSearcherID = searchReq.masterSearcherID
            swarmSearch_msg.order.searchAlgoEnum = searchReq.searchAlgoEnum
            self._swarm_search_publisher.publish(swarmSearch_msg)
            time.sleep(SwarmManager.TIMING_DELAY) # give the swarm search message a second to take
            success = self._ctlr_select_srv_proxy(enums.SWARM_SEARCH_CTRLR).result

        except Exception as ex:
            self.log_warn("Swarm Search exception: " + str(ex))

    # Activates the swarm sequenced takeoff behavior.
    # @param srvReq service request message (not used for this behavior)
    # @return Boolean value indicating behavior initiation success or failure
    def _activate_sequence_takeoff(self, srvReg):
        success = False

        try:
            init = self._init_takeoff_sequencer_srv_proxy(srvReg.setting).result
            if not init:
                self.log_warn("Failed to initialize Swarm Takeoff Sequencer")
                raise Exception("Failed to initialize Swarm Takeoff Sequencer")

            success = self._ctlr_select_srv_proxy(enums.TAKEOFF_SEQUENCE_CTRLR).result

        except Exception as ex:
            self.log_warn("Takeoff sequencer start exception: " + str(ex))

        return success

    # Goes through the list of swarm uav records and returns a list of those
    # that are assigned to a particular subswarm
    # @param subswarm_id: ID of the subswarm being requested
    # @return a list of records for UAVs assigned to the requested subswarm
    def _get_subswarm(self, subswarm_id):
        result = set()
        for uav in self._swarm_uav_states:
            if self._swarm_uav_states[uav].subswarm_id == subswarm_id:
                result.add(self._swarm_uav_states[uav])
        return result

    #-----------------------------------------------------
    # ROS service implementation functions for this object
    #-----------------------------------------------------

    # Handle manual swarm state change service requests.  This service
    # is intended to provide a means of forcing the swarm state to a
    # particular value for testing purposes.  During a normal mission,
    # swarm state will flow according to the mission's conduct.
    # WARNING: this method can force the swarm manager into states that
    #     conflict with the normal mission flow (e.g., assigned to a subswarm
    #     in the INGRESS state or in subswarm 0 in the SWARM_ACTIVE state).
    #     None of these inconsistencies should lead to an unsafe condition.
    # @param stateSrv: service object indicating the new swarm state
    # @return a boolean message response indicating success or failure
    def _process_set_swarm_state(self, stateSrv):
        success = False
        if stateSrv.setting in enums.STATE_STRINGS and \
           stateSrv.setting != enums.SWARM_ACTIVE:
            self._set_swarm_state(stateSrv.setting)
            success = True
            self.log_warn("swarm state force-reset to %s" \
                          %enums.STATE_STRINGS[stateSrv.setting])
        else:
            self.log_warn("attempt to force-reset to invalid swarm state:  %d" \
                          %stateSrv.setting)
        return apsrv.SetIntegerResponse(success)


    # Handles service calls to the run_swarm_sequence_land service.   This
    # method is essentially a pass-through that calls the
    # _process_set_swarm_behavior method with the appropriate parameters for
    # safe activation of the swarm behavior
    # @param ldgSrv: service object containing desired behavior parameters
    # @return a boolean message response indicating success or failure
    def _process_set_sequence_land(self, ldgSrv):
        return self._process_set_swarm_behavior(ldgSrv, \
                    behaviorID = enums.SWARM_SEQUENCE_LAND)

    # Handles service calls to the run_swarm_sequence_takeoff service.   This
    # method is essentially a pass-through that calls the
    # _process_set_swarm_behavior method with the appropriate parameters for
    # safe activation of the swarm behavior
    # @param takeoffSrv: service object containing desired behavior parameters
    # @return a boolean message response indicating success or failure
    def _process_set_sequence_takeoff(self, takeoffSrv):
        return self._process_set_swarm_behavior(takeoffSrv, \
                    behaviorID = enums.SWARM_SEQUENCE_TAKEOFF)

    def _process_set_fixed_formation(self, formSrv):
        success = self._process_set_swarm_behavior(formSrv, \
                       behaviorID = enums.SWARM_FIXED_FORMATION)
        return apsrv.SetSwarmFormationResponse(success.result)

    def _process_set_swarm_search(self, searchSrv):
        success = self._process_set_swarm_behavior(searchSrv, \
                       behaviorID = enums.SWARM_SEARCH)
        return apsrv.SetSwarmSearchRequestResponse(success.result)

    # Handle swarm behavior activation service requests.  The method can
    # activate a parameter-specified behavior (the activateSrv method
    # will then be used by the activating function to incorporate any
    # required parameters) or a service-specified behavior (in which case
    # the activating function shouldn't require any parameters
    # @param activateSrv: message indicating which mode to activate
    # @param behaviorID: ID of behavior to activate (None uses srv value)
    # @return a boolean message response indicating success or failure
    def _process_set_swarm_behavior(self, activateSrv, behaviorID=None):
        success = False
        activateBehavior = behaviorID
        if activateBehavior == None:  activateBehavior = activateSrv.setting

        # Handle special cases first

        # TAKEOFF - negotiates takeoff staging
        if (activateBehavior == enums.SWARM_SEQUENCE_TAKEOFF):
            if (self._swarm_state == enums.INGRESS):
                init_method = self._behavior_activators[activateBehavior]
                success = init_method(activateSrv)
            else:
                self.log_warn("Takeoff Sequence not available when not in Ingress")

        # EGRESS
        elif ((activateBehavior == enums.SWARM_EGRESS) and \
            ((self._swarm_state == enums.INGRESS) or \
             (self._swarm_state == enums.SWARM_READY) or \
             (self._swarm_state == enums.SWARM_ACTIVE) or \
             (self._swarm_state == enums.EGRESS))):
            init_method = self._behavior_activators[activateBehavior]
            success = init_method(activateSrv)

        # Regular swarming behaviors (must be SWARM_READY or SWARM_ACTIVE)
        elif ((activateBehavior in self._behavior_activators) and \
              ((self._swarm_state == enums.SWARM_READY) or \
               (self._swarm_state == enums.SWARM_ACTIVE))):
            init_method = self._behavior_activators[activateBehavior]
            success = init_method(activateSrv)

        # Post-egress behaviors (must be EGRESS)
        elif ((activateBehavior in self._egress_behavior_activators) and \
              (self._swarm_state == enums.EGRESS)):
            init_method = self._egress_behavior_activators[activateBehavior]
            success = init_method(activateSrv)

        # Error cases (can't initiate the requested behavior)
        elif ((activateBehavior not in self._behavior_activators) and \
              ((self._swarm_state == enums.SWARM_READY) or \
               (self._swarm_state == enums.SWARM_ACTIVE))):
            self.log_warn("requested behavior (%d) not available for normal swarming"\
                          %activateBehavior)

        elif ((activateBehavior not in self._egress_behavior_activators) and \
              (self._swarm_state == enums.EGRESS)):
            self.log_warn("requested behavior (%d) not available for post-egress swarming"\
                          %activateBehavior)

        else:
            self.log_warn("vehicle in swarm state %d; cannot initiate swarm behavior %d"\
                          %(self._swarm_state, activateBehavior))

        if success:
            if activateBehavior == enums.SWARM_STANDBY:
                self._set_swarm_state(enums.SWARM_READY)
            elif activateBehavior == enums.SWARM_SEQUENCE_TAKEOFF:
                self._set_swarm_state(enums.INGRESS)
            elif activateBehavior != enums.SWARM_EGRESS:
                self._set_swarm_state(enums.SWARM_ACTIVE)
            self.log_dbug("successful activation of swarm behavior %d" %activateBehavior)
        else:
            self.log_warn("failed to activate swarm behavior %d" %activateBehavior)

        if success:  self._set_swarm_behavior(activateBehavior)
        return apsrv.SetIntegerResponse(success)


    #------------------------------------------
    # ROS Subscriber callbacks -for this object
    #------------------------------------------

    # Handle incoming swarm_uav_states messages
    # @param swarmMsg: message containing swarm data (SwarmStateStamped)
    def _process_swarm_uav_states(self, swarmMsg):
        self._swarm_uav_states.clear()
        del self._swarm_keys[:]
        del self._subswarm_keys[:]
        for vehicle in swarmMsg.swarm:
            self._swarm_uav_states[vehicle.vehicle_id] = vehicle
            self._swarm_keys.append(vehicle.vehicle_id)
            if vehicle.subswarm_id == self._subswarm_id:
                self._subswarm_keys.append(vehicle.vehicle_id)


    # Process recv_subswarm messages.  Messages will be ignored if the
    # vehicle's swarm_state is anything other than SWARM_READY, in which case
    # the new swarm assignment will be published to the update_swwarm topic
    # @param subswarmMsg: message containing new subswarm assignment
    def _process_subswarm_update(self, subswarmMsg):
        # If vehicle in no-set-subswarm state, just return
        if self._swarm_state != enums.SWARM_READY and \
           self._swarm_state != enums.EGRESS:
            self.log_warn("Cannot assign to subswarm %d in swarm state %s" \
                          %(subswarmMsg.data, enums.STATE_STRINGS[self._swarm_state]))
            return

        # If vehicles in a different state than this one are already
        # assigned to the requested subswarm, just return
        subswarm_uavs = self._get_subswarm(subswarmMsg.data)
        for uav in subswarm_uavs:
            if uav.swarm_state != self._swarm_state and \
               subswarmMsg.data != 0:
                self.log_warn("Subswarm %d already in use by incompatible state aircraft" \
                              %subswarmMsg.data)
                return

        # If we get here, it's OK to make the switch
        self._set_subswarm(subswarmMsg.data)


    # Process autopilot status messages.  Messages are tested primarily to
    # detect swarm_state changes as follows:
    #    PRE_FLIGHT or FLIGHT_READY to INGRESS when rel_alt exceeds 20m
    #    INGRESS to SWARM_READY when mis_cur goes to SWARM_STANDBY_WP or higher
    #    Anything to LANDING when the current waypoint type is "land here"
    #    LANDING to ON_DECK when as_read <= 5.0
    # @param statusMsg: Status message
    #

    def _process_autopilot_status(self, statusMsg):
        # Transition to "INGRESS" based on altitude
        if ((self._swarm_state == enums.PRE_FLIGHT) or \
            (self._swarm_state == enums.FLIGHT_READY)):
            if statusMsg.alt_rel > SwarmManager.LAUNCH_ALT:
                self._set_swarm_state(enums.INGRESS)

        # NOTE:  This check is hardcoded as a fixed waypoint ID for now.  Will
        #        need to change to remove dependence on mission file organization
        elif (self._swarm_state == enums.INGRESS):
            if (statusMsg.mis_cur == enums.TAKEOFF_WP):
                setIntSrv = apsrv.SetInteger()
                setIntSrv.setting = enums.INGRESS_LOITER_WP
                self._process_set_sequence_takeoff(setIntSrv)

            if (statusMsg.mis_cur == enums.SWARM_STANDBY_WP):
                self._set_swarm_state(enums.SWARM_READY)
                self._set_swarm_behavior(enums.SWARM_STANDBY)

        # Check for autopilot transition out of auto mode while swarming
        # if this happens, set subswarm 0, swarm state standby, and deactivate controller
        if ((self._subswarm_id != 0) and \
            (statusMsg.mode != enums.AUTO)):
            self._set_subswarm(0)
            self._set_swarm_behavior(enums.SWARM_STANDBY)
            self._set_swarm_state(enums.AP_ERROR)
            self._ctlr_select_srv_proxy(enums.NO_PAYLOAD_CTRL)
            self.log_warn("Invalid autopilot mode (%d) for swarming" %statusMsg.mode)
            return # The rest of this isn't required

        try:
            # Capture the type and altitude of waypoint that is current if required
            if not statusMsg.mis_cur in self._waypoint_types:
                wpt = self._wp_getrange_srv_proxy(statusMsg.mis_cur, \
                                                  statusMsg.mis_cur).points[0]
                self._waypoint_types[statusMsg.mis_cur] = wpt.command
                own_ac = self._swarm_uav_states[self._ownID]
                msl_offset = own_ac.state.pose.pose.position.alt -\
                             own_ac.state.pose.pose.position.rel_alt
                self._waypoint_alts[statusMsg.mis_cur] = wpt.z + msl_offset
                self.log_dbug("Processed new waypoint id " + str(statusMsg.mis_cur) + \
                              ", type " + str(self._waypoint_types[statusMsg.mis_cur]) + \
                              ", altitude " + str(self._waypoint_alts[statusMsg.mis_cur]))

            self._crnt_wpt_alt = self._waypoint_alts[statusMsg.mis_cur]

            if (self._swarm_state == enums.LANDING):
                if (statusMsg.as_read <= 5.0):
                    self._set_swarm_state(enums.ON_DECK)

            # Can transition to LANDING at any time after launch
            # NOTE:  This check is hardcoded as a fixed waypoint ID for now.  Will
            #        need to change to remove dependence on mission file organization
            elif (self._waypoint_types[statusMsg.mis_cur] == enums.WP_TYPE_LAND) and\
                 (self._swarm_state != enums.ON_DECK):
                self._set_swarm_state(enums.LANDING)
                self._set_subswarm(0)

        except Exception as ex:
            self.log_warn('Exception while getting  data for waypoint # ' +\
                          str(statusMsg.mis_cur) + ': ' + str(ex))


#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    swarm_manager = SwarmManager("swarm_manager")

    # Register normal swarm behavior activation methods
    swarm_manager.register_swarm_behavior(enums.SWARM_STANDBY, \
                                          swarm_manager._activate_swarm_standby)
    swarm_manager.register_swarm_behavior(enums.SWARM_FIXED_FORMATION, \
                                          swarm_manager._activate_fixed_follow)
    swarm_manager.register_swarm_behavior(enums.SWARM_SEARCH, \
                                          swarm_manager._activate_swarm_search)
    swarm_manager.register_swarm_behavior(enums.SWARM_EGRESS, \
                                          swarm_manager._activate_egress)
    swarm_manager.register_swarm_behavior(enums.SWARM_SEQUENCE_TAKEOFF, \
                                          swarm_manager._activate_sequence_takeoff)

    # Register post-egress swarm behavior activation methods
    swarm_manager.register_swarm_behavior(enums.SWARM_STANDBY, \
                                          swarm_manager._activate_egress, True)
    swarm_manager.register_swarm_behavior(enums.SWARM_EGRESS, \
                                          swarm_manager._activate_egress, True)
    swarm_manager.register_swarm_behavior(enums.SWARM_SEQUENCE_LAND, \
                                          swarm_manager._activate_sequence_land, True)


    swarm_manager.runAsNode(10.0, [], [], [])
