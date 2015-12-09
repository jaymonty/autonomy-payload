#!/usr/bin/env python

# Object that manages swarm behaviors for a single vehicle.
#
# Duane Davis 2015

# Standard python library imports
import sys
import math
import time
import threading

# ROS library imports
import rospy
import std_msgs.msg as stdmsg

# ACS imports
import autopilot_bridge.srv as bridgesrv
import autopilot_bridge.msg as bridgemsg
import ap_msgs.msg as apmsg
import ap_srvs.srv as apsrv
import ap_lib.nodeable as nodeable
import ap_lib.ap_enumerations as enums


class BehaviorRecord(object):
    ''' Container object containing activation info for a swarm behavior
    The object sets up and holds the ROS service proxies required to set,
    activate, deactivate, pause, and resume behavior operation.  The proxies
    are for the convenience of the owning process, so all class variables
    are public.  Behaviors that do not require activation (should just be
    the "standby" behavior) will not instantiate any proxies.
    No class methods are provided (or required).

    Class member variables:
      id: ID of the behavior
      name: string name of the behavior
      is_wp_behavior: True if the controller inherits from WaypointBehavior
      status: container for received status reports
      status_time: ROS time that the last behavior status update was received
      set_proxy: ROS service proxy used to set the behavior parameters
      run_proxy: ROS service proxy used to activate/deactivate the behavior
      pause_proxy: ROS service proxy used to pause/resume the behavior
    '''

    def __init__(self, behavior_id, behavior_name, is_wp_bhvr, own_node=True):
        ''' Initializes a newly registered behavior
        Registration requires setting up the service that will be used by the
        instantiating object to activate the behavior and setting up the
        service proxy for the behavior node's activation.
        The method also sets up the service used to activate the behavior.
        the activation service is essentially a passthrough that will
        @param behavior_id: integer identifier of the behavior
        @param behavior_name: name of the behavior (used for service names)
        @param is_wp_bhvr: True if the behavior inherits from WaypointBehavior
        @param own_node: True if the behavior is implemented as a node
        '''
        self.id = behavior_id
        self.name = behavior_name
        self.is_wp_bhvr = is_wp_bhvr
        self.status = None
        self.status_time = None

        if own_node:
            self.set_proxy = \
                rospy.ServiceProxy("%s/set" %behavior_name, apsrv.SetBehavior)
            self.run_proxy = \
                rospy.ServiceProxy("%s/run" %behavior_name, apsrv.SetBoolean)
            self.pause_proxy = \
                rospy.ServiceProxy("%s/pause" %behavior_name, apsrv.SetBoolean)
        else:
            self.set_proxy = None
            self.run_proxy = None
            self.pause_proxy = None


#------------------------------------------------------------------------------

class SwarmController(nodeable.Nodeable):
    ''' Object exercising control over swarm participation for one UAV
    This object manages the safe activation and deactivation of swarm
    behaviors.  Individual behaviors register with the ROS node in which
    this object runs using the register_behavior service.

    Class member variables:
      _own_id: ID of this UAV
      _subswarm_id: ID of the subswarm that this UAV is a member of
      _swarm_state: current swarm state of this UAV
      _swarm_behavior: currently active swarm behavior for this UAV
      _swarm_update_time: time of the most recent swarm update
      _swarm_keys: set of IDs for all participating swarm UAVs
      _subswarm_keys: set of IDs for all UAVs assigned to the same subswarm
      _swarm: dictionary containing swarm aircraft state entries
      _ap_status_time: timestamp of the most recent autopilot status
      _ap_waypoint: number of the current autopilot waypoint
      _ap_mode: current autopilot operating mode
      _ap_rel_alt: current autopilot-reported relative altitude
      _ap_as_read: current autopilot-reported airspeed
      _behavior_records: dictionary object containing registered behavior info
      _behavior_switch_time: ROS time of most recent behavior activation
      _behavior_pub_ctr: counter for behavior summary message publication
      _autopilot_waypoint: most recently autopilot-reported waypoint number
      _autopilot_mode: most recently reported autopilot mode
      _ap_wpt_types: dictionary containing autopilot waypoint type info
      _last_wp: the last waypoint from the mission file (infinite loiter)
      _locks: container for synchronization re-entrant locks (thread safety)
      _wp_getlast_proxy: proxy for the autopilot/wp_getlast ROS service
      _wp_getrange_proxy: proxy for the autopilot/wp_getrange ROS service
      _new_subswarm_publisher: publisher to the subswarm_id topic
      _new_swarm_state_publisher: publisher to the ROS swarm_state topic
      _new_swarm_behavior_publisher: publisher to the ROS swarm_behavior topic
      _wp_goto_publisher: publisher to the ROS autopilot/waypoint_goto topic
      _behavior_summary_publisher: publishes to the ROS behavior_summary topic

    Inherited from Nodeable:
      nodeName:  name of the node to start or node in which the object is
      timer: ROS rate object that controls the timing loop
      DBUG_PRINT: set true to force screen debug messages (default FALSE)
      INFO_PRINT: set true to force screen info messages (default FALSE)
      WARN_PRINT: set false to force screen warning messages (default FALSE)

    Class member functions:
      publisherSetup: sets up ROS publishers (inherited from Nodeable)
      serviceSetup: sets up ROS services (inherited from Nodeable)
      serviceProxySetup: sets up ROS service proxies (inherited from Nodeable)
      callbackSetup: sets up ROS topic callbacks (inherited from Nodeable)
      publisherSetup: sets up ROS message publishers (inherited from Nodeable)
      executeTimedLoop: one control loop iteration (inherited from Nodeable)
      _register_behavior: implements the ROS register_behavior service
      _run_behavior: implements the ROS run_behavior service
      _pause_behavior: implements the ROS pause_behavior service
      _set_subswarm_svc: implements the ROS set_subswarm service
      _set_swarm_state_svc: implements the ROS set_swarm_state service
      _process_swarm_summary: callback for the ROS swarm_uav_state topic
      _process_behavior_status: callback for the ROS behavior_status topic
      _process_ap_status: callback for the ROS autopilot/status topic
      _deactivate_all_behaviors: resets the UAV to the swarm standby state
      _activate_new_swarm_behavior: activates a new swarm behavior
      _pre_activation_checks: performs safety checks before behavior activation
      _check_swarm_state: checks (and updates) the swarm state
      _get_subswarm: returns a set of records for all UAVs of a given subswarm
      _update_subswarm: rqd actions upon assignment to a new subswarm
      _update_swarm_state: rqd actions upon setting to a new swarm state
      _update_swarm_behavior: rqd actions when new swarm behavior is activated
      '''

    # Class-specific constants
    LAUNCH_ALT = 20000.0      # AGL (m * 1000) altitude used to detect "launch"
    STALE_BHVR_TIME = rospy.Duration(6.0)
    BHVR_SWITCH_LOCKOUT = rospy.Duration(3.0) # Post-switch time to ignore errors
    BHVR_PUB_INTERVAL = 10   # Timed loops between behavior summary message pub
    NO_FLY_ASPD = 5.0  # Airspeed below which the UAV is assumed "on deck"

    # Enumeration for different locks
    SWARM = 1
    BEHAVIORS = 2
    WAYPOINTS = 3


    AIRBORNE_STATES = set()
    AIRBORNE_STATES.add(enums.INGRESS)
    AIRBORNE_STATES.add(enums.SWARM_READY)
    AIRBORNE_STATES.add(enums.LANDING)
    AIRBORNE_STATES.add(enums.AP_ERROR)

    MAN_RESET_STATES = set()
    MAN_RESET_STATES.add(enums.PRE_FLIGHT)
    MAN_RESET_STATES.add(enums.FLIGHT_READY)
    MAN_RESET_STATES.add(enums.INGRESS)
    MAN_RESET_STATES.add(enums.SWARM_READY)
    MAN_RESET_STATES.add(enums.LANDING)


    def __init__(self, nodename):
        ''' Class initializer sets initial values for member variables
        Assumes that the object is already running within an initialized ROS
        node (i.e., the object does not initialize itself as a node).
        @param nodeName: name of the ROS node for this object
        '''
        nodeable.Nodeable.__init__(self, nodename)
        self._own_id = rospy.get_param("aircraft_id")
        self._subswarm_id = 0
        self._swarm_state = enums.PRE_FLIGHT
        self._swarm_behavior = enums.SWARM_STANDBY
        self._swarm_update_time = None
        self._swarm_keys = set()
        self._subswarm_keys = set()
        self._swarm = dict()
        self._ap_status_time = None
        self._ap_waypoint = 0
        self._ap_mode = 0
        self._ap_rel_alt = 0
        self._ap_as_read = 0
        self._ap_wpt_types = dict()
        self._behavior_records = dict()
        self._behavior_records[enums.SWARM_STANDBY] = \
            BehaviorRecord(enums.SWARM_STANDBY, "standby", False, False)
        self._behavior_switch_time = None
        self._behavior_pub_ctr = 0
        self._autopilot_waypoint = None
        self._ap_mode = None
        self._last_wp = None
        self._locks = dict()
        self._locks[SwarmController.SWARM] = threading.RLock()
        self._locks[SwarmController.BEHAVIORS] = threading.RLock()
        self._locks[SwarmController.WAYPOINTS] = threading.RLock()

        # ROS objects to be instantiated later
        self._wp_getlast_proxy = None
        self._wp_getrange_proxy = None
        self._new_subswarm_publisher = None
        self._new_swarm_state_publisher = None
        self._new_swarm_behavior_publisher = None
        self._wp_intent_publisher = None
        self._wp_goto_publisher = None
        self._behavior_summary_publisher = None

#        self.DBUG_PRINT = True
#        self.INFO_PRINT = True
#        self.WARN_PRINT = True


    #----------------------------------------------
    # Nodeable class virtual method implementations
    #----------------------------------------------

    def serviceSetup(self):
        ''' Creates required services for this object
        The object utilizes a register_behavior service to allow swarm
        behaviors to register themselves and a set_subswarm service to
        assign the UAV to a specific subswarm, and a set_swarm_state service
        to manually assign the UAV to a specific swarm state if required.
        '''
        self.createService("register_behavior", apsrv.RegisterBehavior, \
                           self._register_behavior)
        self.createService("run_behavior", apsrv.SetBehavior, \
                           self._run_behavior)
        self.createService("pause_behavior", apsrv.SetBoolean,
                           self._pause_behavior)
        self.createService("set_subswarm", apsrv.SetInteger, \
                           self._set_subswarm_svc)
        self.createService("set_swarm_state", apsrv.SetInteger, \
                           self._set_swarm_state_svc)


    # Initializes ROS service proxies for this class.  This class requires
    # proxies to call the wp_getall and wp_getlast services to retrieve
    # waypoints from the autopilot
    def serviceProxySetup(self):
        ''' Sets up required ROS service proxies
        This object uses the wp_getlast and wp_getrange services to get
        waypoint information for the current and infinite loiter waypoints.
        '''
        self._wp_getlast_proxy = \
            self.createServiceProxy('wp_getlast', bridgesrv.WPGetAll)
        self._wp_getrange_proxy = \
            self.createServiceProxy('wp_getrange', bridgesrv.WPGetRange)


    def callbackSetup(self):
        ''' Sets up required ROS topic subscriptions
        This node subscribes to the swarm summary topic, the behavior status
        topic, and the autopilot status topic.
        '''
        self.createSubscriber("swarm_uav_states", apmsg.SwarmStateStamped, \
                              self._process_swarm_summary)
        self.createSubscriber("behavior_status", apmsg.BehaviorState, \
                              self._process_behavior_status)
        self.createSubscriber("status", bridgemsg.Status,
                              self._process_ap_status)


    def publisherSetup(self):
        ''' Sets up required ROS publishers
        The object publishes to the subswarm_id, swarm_state, swarm_behavior
        and autopilot/waypoint_goto topics.
        '''
        self._new_subswarm_publisher = \
            self.createPublisher("subswarm_id", stdmsg.UInt8, 1, True)
        self._new_swarm_state_publisher = \
            self.createPublisher("swarm_state", stdmsg.UInt8, 1, True)
        self._new_swarm_behavior_publisher = \
            self.createPublisher("swarm_behavior", stdmsg.UInt8, 1, True)
        self._wp_intent_publisher = \
            self.createPublisher("ap_intent", bridgemsg.Waypoint, 1, True)
        self._wp_goto_publisher = \
            self.createPublisher("waypoint_goto", stdmsg.UInt16, 1, False)
        self._behavior_summary_publisher = \
            self.createPublisher("behavior_summary", \
                                 apmsg.BehaviorGroupStateStamped, 1, False)

        # Publish one message now to initialize the latched publishers with "0"
        self._update_subswarm(self._subswarm_id)
        self._update_swarm_state(self._swarm_state)
        self._update_swarm_behavior(self._swarm_behavior)


    def executeTimedLoop(self):
        ''' Executes one iteration of the control loop
        Not much required (most functionality is in callbacks and services),
        so this just conducts safety checks to make sure everything is as it
        should be, checks for changes to the swarm state, and publishes
        periodic state behavior summary messages.
        '''
        behavior = self._behavior_records[self._swarm_behavior]
        if behavior.id != enums.SWARM_STANDBY and \
           (rospy.Time.now() - \
            behavior.status_time) > SwarmController.STALE_BHVR_TIME:
            self.log_warn("Active behavior (%d) might have stopped" \
                          %self._swarm_behavior)
            self._deactivate_all_behaviors()

        self._check_swarm_state()

        if self._behavior_pub_ctr >= SwarmController.BHVR_PUB_INTERVAL:
            self._publish_behavior_summary()
        else:
            self._behavior_pub_ctr += 1


    #-----------------------------------
    # ROS service implementation methods
    #-----------------------------------

    def _register_behavior(self, reg_srv):
        ''' The register_behavior service for swarm behavior registry
        @param register_srv: service call with the registry information
        @return boolean indication of registry success or failure
        '''
        success = False
        try:
            record = BehaviorRecord(reg_srv.id, reg_srv.name, reg_srv.wpt_ctrl)
            self._behavior_records[reg_srv.id] = record
            success = True
            self.log_info("Behavior %s successfully registered with ID %d"\
                          %(reg_srv.name, reg_srv.id))
        except Exception as ex:
            self.log_warn("Behavior registration exception for behavior %s: %s"\
                          %(reg_srv.name, str(ex)))
        return apsrv.RegisterBehaviorResponse(success)


    def _run_behavior(self, run_srv):
        ''' The run_behavior service implementation
        The run_behavior service will forward the SetBehavior request to the
        appropriate behavior to set the behavior's parameters.  If the
        requested parameters are valid, the vehavior will be activated.
        @param run_srv: service call with the behavior parameters
        @return boolean indication of success or failure
        '''
        try:
            if not run_srv.params.id in self._behavior_records:
                self.log_warn("Attempt to activate unregistered behavior: %d"\
                              %run_srv.params.id)
                return apsrv.SetBehaviorResponse(False)

            if self._subswarm_id == 0:
                self.log_warn("Attempt to activate swarm behavior for subswarm 0")
                return apsrv.SetBehaviorResponse(False)

            behavior = self._behavior_records[run_srv.params.id]
            if behavior.set_proxy:    # Behavior running in its own node
                activation_steps_ok = True
                if self._swarm_behavior != run_srv.params.id:
                    activation_steps_ok = \
                        self._pre_activation_checks(run_srv.params.id)
                if activation_steps_ok:
                    activation_steps_ok = \
                        behavior.set_proxy(run_srv.params).result
                if activation_steps_ok:
                    activation_steps_ok = \
                        self._activate_new_behavior(behavior)
                if activation_steps_ok:
                    self.log_info("%s behavior successfully activated"\
                                  %behavior.name)
                else:
                    self._deactivate_all_behaviors()
                    self.log_warn("Failed to activate behavior %s"\
                                  %behavior.name)

            else:   # Request for swarm standby behavior
                self._deactivate_all_behaviors()
                self.log_info('Swarm behavior set to "Standby"')

        except Exception as ex:
            self.log_warn("Failed to activate behavior %s: %s"\
                          %(behavior.name, str(ex)))

        success = self._swarm_behavior == run_srv.params.id
        return apsrv.SetBehaviorResponse(success)


    def _pause_behavior(self, pause_srv):
        ''' Implements service to pause and resume the active behavior
        @param pause_srv:  SetBoolean with the requested pause state
        @return the post-execution paused status of the current behavior
        '''
        paused = False
        behavior = self._behavior_records[self._swarm_behavior]
        proxy = behavior.pause_proxy
        if proxy:
            paused = proxy(pause_srv.enable)
            paused = paused.result
            self.log_info("Behavior %s pause set to %s"\
                          %(behavior.name, str(paused)))
        else:
            self.log_warn("Attempt to pause unpausable active behavior: %s"\
                          %behavior.name)
        return apsrv.SetBooleanResponse(paused)


    def _set_subswarm_svc(self, subswarm_srv):
        ''' Implements the service to assign the UAV to a subswarm
        @param subswarm_srv: SetInteger with the requested subswarm ID
        @return True if the UAV was successfully assigned to the subswarm
        '''
        if self._subswarm_id == subswarm_srv.setting:
            self.log_info("Attempt to reassign to current subswarm: %d"\
                          %self._subswarm_id)
            return apsrv.SetIntegerResponse(True)

        if self._swarm_state != enums.SWARM_READY:
            self.log_warn("Attempt to change subswarm while in swarm state %s"\
                          %enums.STATE_STRINGS[self._swarm_state])
            return apsrv.SetIntegerResponse(False)

        if self._swarm_behavior != enums.SWARM_STANDBY:
            self.log_warn("Attempt to change subswarm with active swarm behavior: %s"\
                          %self._behavior_records[self._swarm_behavior].name)
            return apsrv.SetIntegerResponse(False)

        if subswarm_srv.setting == 0:
            self._update_subswarm(0)
            self.log_info("UAV reassigned to subswarm 0")
            return apsrv.SetIntegerResponse(True)

        records = self._get_subswarm(subswarm_srv.setting)
        for uav in records:
            if uav.swarm_behavior != enums.SWARM_STANDBY or \
               uav.swarm_state != enums.SWARM_READY:
                self.log_warn("Attempt to subswarm %d in incompatible state"\
                              %subswarm_srv.setting)
                return apsrv.SetIntegerResponse(False)

        self._update_subswarm(subswarm_srv.setting)
        return apsrv.SetIntegerResponse(True)


    def _set_swarm_state_svc(self, state_srv):
        ''' Implements the service to manually reset the UAV's swarm state
        NOTE:  This method is not for routine use--normal swarm state
               transitions are detected automatically.
        @param subswarm_srv: SetInteger with the requested new swarm state
        @return True if the UAV swarm state was successfully reset
        '''
        if state_srv.setting in SwarmController.MAN_RESET_STATES:
            self.log_warn("Manual reset to new swarm state: %s"\
                          %enums.STATE_STRINGS[state_srv.setting])
            self._deactivate_all_behaviors()
            self._update_subswarm(0)
            self._update_swarm_state(state_srv.setting)
            return apsrv.SetIntegerResponse(True)

        self.log_warn("Attempt to set swarm state to invalid value: %d"\
                      %state_srv.setting)
        return apsrv.SetIntegerResponse(False)


    #---------------------------------
    # ROS topic subscription callbacks
    #---------------------------------

    def _process_swarm_summary(self, swarm_msg):
        ''' Processes swarm_uav_state (swarm summary) messages
        @param swarm_msg:  SwarmStateStamped message with swarm summary info
        '''
        with self._locks[SwarmController.SWARM]:
            self._swarm_update_time = swarm_msg.header.stamp
            self._swarm.clear()
            self._swarm_keys.clear()
            self._subswarm_keys.clear()

            for uav in swarm_msg.swarm:
               uav_id = uav.vehicle_id
               self._swarm[uav_id] = uav
               self._swarm_keys.add(uav_id)
               if uav.subswarm_id == self._subswarm_id:
                   self._subswarm_keys.add(uav_id)


    def _process_behavior_status(self, state_msg):
        ''' Processes behavior state messages from individual behaviors
        @param state_msg:  BehaviorState message with state info
        '''
        try:
            with self._locks[SwarmController.BEHAVIORS]:
                bhvr = self._behavior_records[state_msg.behavior_id]
                bhvr.status = state_msg
                bhvr.status_time = rospy.Time.now()

                if bhvr.status.is_active and \
                   bhvr.id != self._swarm_behavior and \
                   (bhvr.status_time - self._behavior_switch_time) >\
                       SwarmController.BHVR_SWITCH_LOCKOUT:
                    self.log_warn("Unexpected behavior (%d) reporting active"\
                                  %state_msg.behavior_id)
                    self._deactivate_all_behaviors()

                elif not bhvr.status.is_active and \
                     bhvr.id == self._swarm_behavior and \
                     (bhvr.status_time - self._behavior_switch_time) >\
                         SwarmController.BHVR_SWITCH_LOCKOUT:
                    self.log_warn("Behavior %d unexpected inactive report"\
                                  %state_msg.behavior_id)
                    self._deactivate_all_behaviors()

        except Exception as ex:
            self.log_warn("Callback Exception for behavior %d: %s"\
                          %(state_msg.behavior_id, str(ex)))


    def _process_ap_status(self, status_msg):
        ''' Processes autopilot status messages
        @param status_msg: autopilot_bridge/Status message
        '''
        if self._ap_waypoint != status_msg.mis_cur:
            with self._locks[SwarmController.WAYPOINTS]:
                wpt = self._wp_getrange_proxy(status_msg.mis_cur, \
                                              status_msg.mis_cur)
                if wpt.ok:
                    self._wp_intent_publisher.publish(wpt.points[0])
                    self._ap_wpt_types[status_msg.mis_cur] = \
                        wpt.points[0].command
                    self._ap_waypoint = status_msg.mis_cur

        self._ap_status_time = status_msg.header.stamp
        self._ap_mode = status_msg.mode
        self._ap_rel_alt = status_msg.alt_rel
        self._ap_as_read = status_msg.as_read


    #-----------------------
    # Class-specific methods
    #-----------------------

    def _deactivate_all_behaviors(self):
        ''' Deactivates all swarm behaviors
        Also directs the UAV to the "standby" waypoint
        '''
        self._behavior_switch_time = rospy.Time.now()
        with self._locks[SwarmController.BEHAVIORS]:
            for behavior_id in self._behavior_records:
                behavior = self._behavior_records[behavior_id]
                if behavior.run_proxy:
                    try:
                        behavior.run_proxy(False)
                    except Exception as ex:
                        self.log_warn("Exception deactivating behavior %d: %s" \
                                 %(behavior.id, str(ex)))
            self._update_swarm_behavior(enums.SWARM_STANDBY)
        self._wp_goto_publisher.publish(stdmsg.UInt16(enums.SWARM_STANDBY_WP))


    def _activate_new_behavior(self, behavior):
        ''' Activates a new behavior after deactivating the current one
        @param behavior: record object of the new behavior
        @return True if the behavior was successfully activated
        '''
        if behavior.id == self._swarm_behavior:  # Already active
            return True

        self._deactivate_all_behaviors()
        if not behavior.run_proxy:
            return True

        # Now activate the new one
        self._behavior_switch_time = rospy.Time.now()
        if behavior.is_wp_bhvr:
            self._wp_goto_publisher.publish(stdmsg.UInt16(self._last_wp.seq))
            time.sleep(1.0)  # Give it time to get processed
        success = behavior.run_proxy(True).result

        if success:
            self._update_swarm_behavior(behavior.id)

        return success


    def _pre_activation_checks(self, behavior_id):
        ''' Safety checks to completed before behavior activation
        @param behavior_id: identifier for the behavior to be activated
        @return True if the safety checks are successful
        '''
        with self._locks[SwarmController.BEHAVIORS]:

            if self._swarm_state != enums.SWARM_READY:
                self.log_warn("Not swarm ready: cannot initiate behavior")
                return False

            if not behavior_id in self._behavior_records:
                self.log_warn("Attempt to initialiate unknown behavior: %d"\
                              %behavior_id)
                return False

            behavior = self._behavior_records[behavior_id]

            if ((rospy.Time.now() - \
                 behavior.status_time) > SwarmController.STALE_BHVR_TIME):
                self.log_warn('Attempt to activate "stale" behavior: %s'\
                              %behavior.name)
                return False

            if not self._ap_mode == enums.AUTO:
                self.log_warn("Autopilot must be in AUTO to activate behavior")
                return False

            # Checks for WaypointBehaviors--get through this & things are good
            if behavior.is_wp_bhvr and self._last_wp == None:
                try:
                    self._last_wp = self._wp_getlast_proxy().points
                except Exception as ex:
                    self._log_warn("WP fetch exception: cannot activate behavior %s"\
                                   %behavior.name)
                    self._last_wp = None
                    return False

                if len(self._last_wp) == 0:
                    self.log_warn("Failed WP fetch: cannot activate behavior %s"\
                                  %behavior.name)
                    self._last_wp = None
                    return False

                if self._last_wp[0].command != enums.WP_TYPE_LOITER:
                    self.log_warn("Last WP not loiter: cannot activate behavior %s"\
                                  %behavior.name)
                    self._last_wp = None
                    return False

                self._last_wp = self._last_wp[0]
                rospy.set_param("last_mission_wp_id", self._last_wp.seq)

        return True


    def _check_swarm_state(self):
        ''' Determines when to conduct swarm state changes
        '''
        # Normal state transitions
        if ((self._swarm_state == enums.PRE_FLIGHT) and \
            (rospy.has_param("flight_ready")) and \
            (bool(rospy.get_param("flight_ready")))):
                self._update_swarm_state(enums.FLIGHT_READY)

        elif self._swarm_state == enums.PRE_FLIGHT or \
           self._swarm_state == enums.FLIGHT_READY:
            if self._ap_rel_alt > SwarmController.LAUNCH_ALT:
                self._update_swarm_state(enums.INGRESS)

        elif self._swarm_state == enums.INGRESS:
            if self._ap_waypoint >= enums.SWARM_STANDBY_WP:
                self._update_swarm_state(enums.SWARM_READY)

        elif self._swarm_state == enums.SWARM_READY:
            with self._locks[SwarmController.WAYPOINTS]:
                if self._ap_wpt_types[self._ap_waypoint] == enums.WP_TYPE_LAND:
                    self._update_swarm_state(enums.LANDING)
                    self._update_subswarm(0)

        elif self._swarm_state == enums.LANDING:
            if self._ap_as_read < SwarmController.NO_FLY_ASPD:
                self._update_swarm_state(enums.ON_DECK)

        # Check for error state or unexpected transitions (e.g., crash)
        if self._swarm_state in SwarmController.AIRBORNE_STATES and \
           self._swarm_state != enums.AP_ERROR and \
           self._ap_mode != enums.AUTO:
            self._deactivate_all_behaviors()
            self._update_subswarm(0)
            self._update_swarm_state(enums.AP_ERROR)
            self.log_warn("Autopilot not in AUTO: disabling swarming")

#        if self._swarm_state == enums.AP_ERROR and \
#           self._ap_mode == enums.AUTO:
#            self._update_swarm_state(enums.SWARM_READY)
#            self._deactivate_all_behaviors()
#            self._update_subswarm(0)
#            self.log_info("Autopilot reset to AUTO: swarming re-enabled")

        if self._swarm_state in SwarmController.AIRBORNE_STATES and \
           self._ap_as_read < SwarmController.NO_FLY_ASPD:
            self._deactivate_all_behaviors()
            self._update_subswarm(0)
            self._update_swarm_state(enums.ON_DECK)
            self.log_warn("Unexpected low airspeed--not flying anymore!")


    def _publish_behavior_summary(self):
        ''' Publishes a summary message to the ROS behavior_summary topic
        '''
        status = apmsg.BehaviorGroupStateStamped()
        status.header.stamp = rospy.Time.now()
        status.state.active_behavior = self._swarm_behavior

        with self._locks[SwarmController.BEHAVIORS]:
            for behavior_id in self._behavior_records:
                behavior = self._behavior_records[behavior_id]
                if behavior.status:
                    status.state.behaviors.append(behavior.status)

        self._behavior_pub_ctr = 0
        self._behavior_summary_publisher.publish(status)


    def _get_subswarm(self, subswarm_id=None):
        ''' Returns all UAV records for the requested subswarm
        @param subswarm_id: requested subswarm (default own subswarm)
        @return a set with all requested subswarm records
        '''
        if subswarm_id == None: subswarm_id = self._subswarm_id
        records = set()
        with self._locks[SwarmController.SWARM]:
            for uav_id in self._swarm:
                uav_rec = self._swarm[uav_id]
                if uav_rec.subswarm_id == subswarm_id:
                    records.add(uav_rec)
        return records


    def _update_swarm_state(self, new_state):
        ''' Performs all required actions when the swarm state changes
        @param new_state: new swarm state value
        '''
        self._swarm_state = new_state
        self._new_swarm_state_publisher.publish(stdmsg.UInt8(new_state))
        self.log_info("Swarm state updated to %s" \
                      %enums.STATE_STRINGS[self._swarm_state])


    def _update_subswarm(self, new_subswarm):
        ''' Performs all required actions upon assignment to a new subswarm
        @param new_subswarm: ID of the subswarm to which the UAV is assigned
        '''
        self._subswarm_id = new_subswarm
        self._new_subswarm_publisher.publish(stdmsg.UInt8(new_subswarm))
        self.log_info("Subswarm updated to %d" %self._subswarm_id)


    def _update_swarm_behavior(self, new_behavior):
        ''' Performs all required post-swarm-behavior-initiation actions
        @param new_behavior: behavior_id of the initiated swarm behavior
        '''
        self._swarm_behavior = new_behavior
        self._new_swarm_behavior_publisher.publish(stdmsg.UInt8(new_behavior))
        self._behavior_pub_ctr = SwarmController.BHVR_PUB_INTERVAL
        self.log_info("Active swarm behavior updated to %s"\
                      %self._behavior_records[self._swarm_behavior].name)


#------------------------------------------------------------------------------
# Main code
#------------------------------------------------------------------------------

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    swarm_control = SwarmController("swarm_control")
    swarm_control.runAsNode(10.0)
