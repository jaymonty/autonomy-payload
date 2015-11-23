#!/usr/bin/env python

#-------------------------------------------------------------------------
# Behavior
# Duane Davis, 2015
#
# Defines an abstract class for wrapping ACS behaviors into ROS objects
# that can be incorporated into other classes or run as independent nodes
#-------------------------------------------------------------------------

import threading
import time

# ROS imports
import rospy
from rospy import rostime
import std_msgs.msg as stdmsg
from ap_lib import nodeable
from ap_lib import ap_enumerations as enums
import ap_msgs.msg as apmsg
import ap_srvs.srv as apsrv
import autopilot_bridge.msg as bridgemsg


class Behavior(nodeable.Nodeable):
    ''' Abstract class for defining ACS swarm behavior controllers
    Instantiated objects will provide ROS services for parameter setting
    (nodename/set), activation/deactivation (nodename/run), and
    pause/resume (nodename/pause).

    Class member variables:
      behaviorID: identifier (int) for this particular behavior
      _subswarm_id: ID of the subswarm to which this UAV is assigned
      _swarm: container for state info for all swarm UAVs
      _reds: container for state infor for all red UAVs
      _swarm_keys: key values (IDs) of all swarm UAVs
      _subswarm_keys: key values (IDs) of all subswarm UAVs
      _crashed_keys: key values (IDs) of swarm UAVs suspected of crashing
      _swarm_subscriber: subscriber object for swarm_tracker reports
      _red_subscriber: subscriber object for red_tracker reports
      _ap_wp: current autopilot waypoint ID
      _ap_intent: most recently ordered autopilot waypoint
      _swarm_lock: reentrant lock enforcing thread-safe swarm dictionary access
      _reds_lock: reentrant lock enforcing thread-safe reds dictionary access
      is_ready: set to True when the behavior has been initialized
      is_active: set to True when the behavior is running 
      is_paused: set to True when an active behavior is paused
      _uses_wp_control: set to True if the behavior drives by waypoint
      _statusPublisher: publisher object for behavior status
      _statusStamp: timestamp of the last status message publication
      _sequence: sequence number of the next status message

    Inherited from Nodeable:
      nodeName:  Name of the node to start or node in which the object is
      timer: ROS rate object that controls the timing loop
      DBUG_PRINT: set true to force screen debug messages (default FALSE)
      INFO_PRINT: set true to force screen info messages (default FALSE)
      WARN_PRINT: set false to force screen warning messages (default FALSE)

    Class member functions
      executeTimedLoop: implementation of the Nodeable class virtual function
      set_active: "safe" behavior activation and deactivation
      set_ready_state: "safe" transitions between behavior ready and not ready
      _send_status_message: publishes the behavior's periodic status message
      _set_srv: handler for the behavior's set service
      _activate_srv: handler for the behavior's run service
      _pause_srv: handler for the behavior's pause service
      _process_swarm_uav_states: callback for swarm uav state messages
      _process_red_uav_states: callback for red uav state messages
      _process_upate_subswarm: callback for the update_subswarm topic
      _process_autopilot_status: callback for autopilot status messages
      _process_ap_intent: callback for autopilot intent messages
      subscribe_to_swarm: subscribes to the swarm_uav_states topic
      subscribe_to_reds: subscribes to the red_uav_states topic

    Virtual methods for inheriting class implementation
      run_behavior: implements 1 iteration of the behavior's control loop
      set_behavior: parses the set message and sets behavior parameters
      _safety_checks: conducts behavior-specific online safety checks
    '''

    # Class specific constants and enumerations

    MAX_ACTIVATE_T = rospy.Duration(10.0) # Expected max time for activation

    def __init__(self, nodename, behavior_id):
        ''' Class initializer sets base class member variables
        @param nodename: name of the node that the object is contained in
        @param behavior_id: unique identifier for this behavior (integer)
        '''
        nodeable.Nodeable.__init__(self, nodename)
        self.behaviorID = behavior_id
        self._subswarm_id = 0
        self._swarm = dict()
        self._swarm_keys = set()
        self._subswarm_keys = set()
        self._crashed_keys = set()
        self._reds = dict()
        self._ap_wp = 0
        self._ap_intent = None
        self._uses_wp_control = False
        self.is_ready = False
        self.is_active = False
        self.is_paused = False
        self._statusStamp = None
        self._sequence = 0
        self._swarm_lock = threading.RLock()
        self._reds_lock = threading.RLock()
        rospy.Subscriber("autopilot/status", bridgemsg.Status, \
            self._process_ap_status)
        rospy.Subscriber("swarm_control/subswarm_id", stdmsg.UInt8,
                         self._process_upate_subswarm)
        rospy.Subscriber("swarm_control/ap_intent", bridgemsg.Waypoint, \
                         self._process_ap_intent)
        rospy.Subscriber("network/recv_swarm_data", apmsg.BehaviorParameters, \
                         self._process_swarm_data_msg)
        self._swarm_subscriber = None
        self._red_subscriber = None
        self._statusPublisher = \
            rospy.Publisher("swarm_control/behavior_status", \
                            apmsg.BehaviorState, tcp_nodelay=True, \
                            latch=False, queue_size=1)
        self.createService("set", apsrv.SetBehavior, self._set_srv)
        self.createService("run", apsrv.SetBoolean, self._run_srv)
        self.createService("pause", apsrv.SetBoolean, self._pause_srv)


    #-------------------------------------------------------------------
    # Virtual methods of this class (must be implement in child classes)
    #-------------------------------------------------------------------

    def run_behavior(self):
        ''' Runs one iteration of the behavior's control loop
        This is a "virtual" method that must be implemented by inheriting
        classes.  Implementaqtions will process all behavior-specific data,
        publish any required control commands.  Implementations do not need
        to determine whether the behavior is "active" or not, but must
        incorporate any behavior-specific safety checks (deactivate the
        behavior if it becomes unsafe or unsatisfiable).
        '''
        pass


    def set_behavior(self, paramSvc):
        ''' Sets behavior parameters based on set service parameters
        This is a "virtual" method that must be implemented by inheriting
        classes.  Implementations must parse the paramSvs.params.params
        unsigned byte array and set behavior-specific parameters accordingly.
        NOTE: If the implementing behavior requires swarm data (i.e.,
              swarm_tracker output), the self.subscribe_to_swarm() method MUST
              be called at the beginning of the set_behavior implementation.
        @param paramSvc: SetBehaviorParameters service request
        @return True (SetBehaviorParametersResponse) if set with valid parameters
        '''
        return apsrv.SetBehaviorParametersResponse(False)


    def _safety_checks(self):
        ''' Conducts behavior-specific safety checks
        @return True if the behavior passes all safety checks (False otherwise)
        '''
        return True


    def _process_swarm_data_msg(self, dataMsg):
        ''' Processes swarm data messages received over the network
        Implementing methods will need to parse the bitmapped "params"
        field of the message.  No action should be taken by implementing
        methods unless the behavior is "ready" or "active".
        @param dataMsg: message containing received behavior data
        '''
        pass


    #-----------------------------------------------------------------
    # Parent class virtual method implementations and method overrides
    #-----------------------------------------------------------------

    def executeTimedLoop(self):
        ''' Runs a timed single loop of the behavior
        This method implements the Nodeable class virtual method.  This method
        will invoke the object-specific control method if (and only if) the
        behavior is ready, active, and not paused.  Status messages are
        published by this method at one-second intervals.
        '''
        try:
            if self._statusStamp == None: self._statusStamp = rospy.Time.now()

            # Publish a status message at 1-second intervals
            time = rospy.Time.now()
            interval = (time.secs + (time.nsecs / 1e9)) - \
                       (self._statusStamp.secs + \
                        (self._statusStamp.nsecs / 1e9))
            if interval >= 1.0:
                self._send_status_message()
            if self.is_active and not self._safety_checks():
                self.set_ready_state(False)
                self.log_warn("%s failed behavior-specific safety checks" \
                              %self.nodeName)
            if self.is_ready and self.is_active and not self.is_paused:
                self.run_behavior()
        except Exception as ex:
            self.log_warn("%s, Timed Loop Error: %s" \
                          %(self.nodeName, str(ex)))


    def runAsNode(self, rate=10.0):
        ''' Runs the behavior as an independent node
        Duplicates parent class functionality, but adds behavior registration
        @param rate: rate (hz) of the node's timing loop
        '''
        # Initialize ROS node & set up callbacks, services and publishers
        rospy.init_node(self.nodeName)
        self.serviceSetup()
        self.serviceProxySetup()
        self.callbackSetup()
        self.publisherSetup()
        self.register_behavior()

        # Set up the timing object and start the timing loop
        self.timer = rospy.Rate(rate)
        self.log_dbug("starting " + self.nodeName)
        print "Starting " + self.nodeName + " node"
        while not rospy.is_shutdown():
            self.timer.sleep()
            self.executeTimedLoop()


    #--------------------------------------------------------
    # Class-specific methods implementing class functionality
    #--------------------------------------------------------

    def register_behavior(self):
        ''' Registers the behavior with the swarm controller
        This method is called by the overridden Nodeable.runAsNode method
        '''
        rospy.wait_for_service('swarm_control/register_behavior')
        register = rospy.ServiceProxy('swarm_control/register_behavior', \
                                      apsrv.RegisterBehavior)
        register(self.behaviorID, self.nodeName, self._uses_wp_control)


    def _set_srv(self, set_req):
        ''' Implements the service for setting behavior parameters
        @param set_req:  service (SetBehaviorParameters) request
        @return service response message with ready status
        '''
        resp = self.set_behavior(set_req.params.params)
        self.log_info("%s set request returned %s" %(self.nodeName, str(resp)))
        return apsrv.SetBehaviorResponse(resp)


    def set_active(self, activate):
        ''' Used to activate and deactivate the behavior
        Will not activate an "unready" behavior or one that is already active
        @param activate: Boolean value to activate or deactivate the behavior
        @return active status upon method completion
        '''
        if activate and not self.is_ready:
            self.is_active = False
            self.log_warn("attempt to activate uninitialized behavior %s"\
                          %self.nodeName)
        elif activate and not self.is_active:
            self.is_active = activate
            self.log_info("activating behavior %s"%self.nodeName)
            self._send_status_message()
        elif not activate and self.is_active:
            self.log_info("deactivating behavior %s"%self.nodeName)
            self.set_ready_state(False)
            self._send_status_message()
        return self.is_active


    def _run_srv(self, activate_req):
        ''' Implements the service for activating/deactivating the behavior
        @param activate_req: service call (SetBoolean) message
        @return service response message with the behavior's active status
        '''
        resp = self.set_active(activate_req.enable)
        self.log_info("%s activate request %s returned %s"\
                      %(self.nodeName, str(activate_req.enable), str(resp)))
        return apsrv.SetBooleanResponse(resp)


    def set_ready_state(self, ready):
        ''' Sets the behavior's ready state when new control inputs received
        Publishes an updated behavior status message if the ready state changes
        Will also ensure that the behavior is deactivated if the ready state
        is set to false.
        @param ready: Boolean value to set the behavior's ready state
        @return Boolean ready-state value
        '''
        # Only needs to do anything if the ready state is changing
        if not self.is_ready and ready:
            self.is_ready = True
            self.is_paused = False
            self._send_status_message()
            self.log_info("%s ready state set to 'True'"%self.nodeName)
        elif (self.is_ready or self.is_active) and not ready:
            self.is_ready = False
            self.is_active = False
            if self._swarm_subscriber: self._swarm_subscriber.unregister()
            if self._red_subscriber: self._red_subscriber.unregister()
            self._swarm_subscriber = None
            self._red_subscriber = None
            self._send_status_message()
            self.log_info("%s ready state and active state set to 'False'"\
                          %self.nodeName)
        return self.is_ready


    def set_pause(self, pause):
        ''' Pauses and unpauses the behavior (if active).
        A paused behavior should be able to to be unpaused without being 
        re-initialized (i.e., the behavior is still active, but its control 
        function is just being bypassed).
        NOTE: Does not do anything except change the value of the is_paused
              member variable.  If more is required to achieve correct behavior,
              this method should be overridden by the implementing class.  Call
              this version at the end of the subclass definition.
        @param pause: Boolean value to activate or pause or unpause
        @return Boolean is-paused state
        '''
        if not self.is_active: return False
        if (pause and not self.is_paused) or (not pause and self.is_paused):
            self.is_paused = pause
            self.log_info(" %s is_paused set to %s"%(self.nodeName, str(pause)))
            self._send_status_message()
        return self.is_paused


    def _pause_srv(self, pause_req):
        ''' Implements the service for pausing and unpausing the behavior
        @param activate_req: service call (SetBoolean) message
        @return service response (SetBoolean) message
        '''
        resp = self.set_pause(pause_req.enable)
        self.log_info("pause request %s returned %s"\
                      %(str(pause_req.enable), str(resp)))
        return apsrv.SetBooleanResponse(resp)


    def _send_status_message(self):
        ''' Publishes a BehaviorState message to the behavior status topic
        '''
        self._statusStamp = rospy.Time.now()
        status = apmsg.BehaviorState()
        status.behavior_id = self.behaviorID
        status.sequence = self._sequence
        status.is_ready = self.is_ready
        status.is_active = self.is_active
        status.is_paused = self.is_paused
        self._statusPublisher.publish(status)
        self._sequence += 1


    def subscribe_to_swarm(self):
        ''' Establishes the subscription to the swarm information
        The swarm_uav_states topic is not subscribed to when the behavior is
        inactive in order to minimize the communication requirement.
        NOTE:  This method must be called by an implementing behavior's
               set_behavior method in order to have access to the swarm data.
        '''
        if self._swarm_subscriber == None:
            self._swarm_subscriber = \
                rospy.Subscriber("swarm_tracker/swarm_uav_states", \
                                 apmsg.SwarmStateStamped, \
                                 self._process_swarm_uav_states)
        time.sleep(0.5)  # Give it a little time to get at least one update


    def subscribe_to_red_states(self):
        ''' Establishes the subscription to the red UAV information
        The red_uav_states topic is not subscribed to when the behavior is
        inactive in order to minimize the communication requirement.
        NOTE:  This method must be called by an implementing behavior's
               set_behavior method in order to have access to the red UAV data.
        '''
        if self._red_subscriber == None:
            self._red_subscriber = \
                rospy.Subscriber("red_tracker/red_uav_states", \
                                 apmsg.RedSwarmStateStamped, \
                                 self._process_red_uav_states)
        time.sleep(0.5)  # Give it a little time to get at least one update


    #---------------------------------
    # ROS topic subscription callbacks
    #---------------------------------

    def _process_swarm_uav_states(self, swarmMsg):
        ''' Handle incoming swarm_uav_states messages
        @param swarmMsg: message containing swarm data (SwarmStateStamped)
        '''
        with self._swarm_lock:
            self._swarm.clear()
            self._swarm_keys.clear()
            self._subswarm_keys.clear()
            self._crashed_keys.clear()
            self._crashed_keys.update(swarmMsg.crashed_list)
            for vehicle in swarmMsg.swarm:
                self._swarm[vehicle.vehicle_id] = vehicle
                self._swarm_keys.add(vehicle.vehicle_id)
                if vehicle.subswarm_id == self._subswarm_id:
                    self._subswarm_keys.add(vehicle.vehicle_id)


    def _process_red_uav_states(self, redsMsg):
        ''' Handle incoming red_uav_states messages
        @param redsMsg: message containing swarm data (RedSwarmStateStamped)
        '''
        with self._reds_lock:
            self._reds.clear()
            for uav in redsMsg.reds:
                self._reds[uav.vehicle_id] = uav


    def _process_upate_subswarm(self, subswarmMsg):
        ''' Handle incoming update_subswarm messages
        @param subswarmMsg: message containing the new subswarm assignment
        '''
        self._subswarm_id = subswarmMsg.data


    def _process_ap_status(self, status_msg):
        ''' Processes autopilot status messages
        For now, just makes sure that the autopilot is in AUTO mode
        when the behavior is active, and deactivates it if it is not.
        @param status_msg: autopilot status message
        '''
        if status_msg.mode != enums.AUTO:
            self.is_ready = False
            if self.is_active:
                self.log_warn("Autopilot not in AUTO:  deactivating %s" \
                              %self.nodeName)
                self.set_active(False)
        self._ap_wp = status_msg.mis_cur


    def _process_ap_intent(self, wptMsg):
        ''' Processes autopilot intent messages
        @param wptMsg waypoint message
        '''
        self._ap_intent = wptMsg

