#!/usr/bin/env python

# Object that sequences and controls subswarm aircraft for landing.
#
# Duane Davis 2015

# Standard python library imports
import sys
import threading

# ROS library imports
import rospy
import std_msgs.msg as stdmsg

# ACS imports
import ap_msgs.msg as apmsg
import ap_srvs.srv as apsrv
import ap_lib.ap_enumerations as enums
import ap_lib.nodeable as nodeable
import ap_lib.gps_utils as gps
import ap_lib.waypoint_controller as wp_controller
import autopilot_bridge.srv as apbrgsrv
import autopilot_bridge.msg as apbrgmsg

LAND_WP_DIST = 75.0     # Distance from landing wp to start descent
ALT_BLOCK_CAPTURE = 3.0  # UAV within lower alt block at this alt offset
STACK_LOITER_TIME = rospy.Duration(15.0) # Min time at stack location before descending

# Enumeration for controller states
NEGOTIATE_ORDER = 0
START_TRANSIT = 1
IN_TRANSIT = 2
STACK_ARRIVAL = 3
IN_STACK = 4
ON_FINAL = 5

BASE_WP_LOITER_RAD = 50.0

# Object that sequences and controls subswarm aircraft for landing.
#
# Class member variables:
#   _ownID: this UAV's ID
#   _subswarm_id: subswarm to which this UAV is assigned
#   _leaderID: ID of the UAV to follow in the landing sequence
#   _ordered_wp_id: ID of the ordered waypoint when controller initiated
#   _ldgWptIndex: Index (ID) of the landing waypoint
#   _alt_block: assigned altitude block relative to the base altitude
#   _wp_msg: LLA message object for waypoint orders to be published
#   _swarm_uav_states: dictionary object containing state of all swarm aircraft
#   _swarm_keys: list of all aircraft IDs (keys) in the swarm dictionary
#   _subswarm_keys: list of subswarm aircraft IDs (keys) in the swarm dictionary
#   _ldg_wp_publisher: ROS publisher to direct the UAV to the landing waypoint
#   _ldg_start_publisher: ROS publisher notifying autopilot of landing start or abort
#   _wp_getrange_srv_proxy: ROS proxy for using the autopilot/wp_getrange service
#   _fpr_param_set_srv_proxy: ROS proxy for using the autopilot/fpr_param_set service
#   _controller_state: determines what the controller is currently doing
#   _transit_started: set to True when the transit wpt is issued
#   _transit_complete: set to True when the transit wpt has been reached
#   _landing_ordered: set to True when ordered to the landing wpt
#   _lock: prevent unsafe thread interaction (swarm summary dictionary)
#   _stack_time: ROS time of arrival at the landing stack location
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
#   _process_swarm_uav_states: callback from swarm uav state messages
#
class SwarmLandingSequencer(wp_controller.WaypointController):

    # Class initializer initializes class variables.
    # This assumes that the object is already running within an initialized
    # ROS node (i.e., the object does not initialize itself as a node).
    # This enables multiple objects to run within a single node if
    # required.  The initializer does some parameter checking (mostly
    # whether or not all of the required parameters are there), but does
    # not check ranges, magnitudes, or signs.
    # @param nodename: name of the ROS node in which this object exists
    # @param own_id: ID (integer) of this aircraft
    def __init__(self, nodename, ctlr_id):
        wp_controller.WaypointController.__init__(self, nodename, \
            enums.LANDING_SEQUENCE_CTRLR)
        self._ownID = rospy.get_param("aircraft_id")
        self._ordered_wp_id = 0
        self._leaderID = 0
        self._subswarm_id = 0
        self._ldgWptIndex = 0
        self._alt_block = 0
        self._wp_msg = apbrgmsg.LLA()
        self._swarm_uav_states = dict()
        self._swarm_keys = []
        self._subswarm_keys = []
        self._ldg_wp_publisher = None
        self._ldg_start_publisher = None
        self._wp_getrange_srv_proxy = None
        self._fpr_param_set_srv_proxy = None
        self._controller_state = None
        self._lock = threading.RLock()
        self._stack_time = None
#        self.DBUG_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Activates or deactivates the controller.  This method overrides the
    # Controller class implementation so that the autopilot can be notified
    # when a previously ordered landing has been ordered.  The parent-class
    # function is called after the class-specific processing is complete 
    # to ensure the rest of the safety functionality of the Controller class
    # implementation is retained.
    # @param activate: Boolean value to activate or deactivate the controller
    def set_active(self, activate):
        if not activate and self.is_active and \
           self._controller_state == ON_FINAL:
            self._ldg_start_publisher.publish(False)
        return super(SwarmLandingSequencer, self).set_active(activate)


    # Creates a publisher to send the vehicle to the appropriate waypoint ID
    # to initiate landing once it's this aircraft's turn
    # @param params: list or required parameters (none for now)
    def publisherSetup(self, params=[]):
        self._ldg_wp_publisher = \
            self.createPublisher("waypoint_goto", stdmsg.UInt16, 1)
        self._ldg_start_publisher = \
            self.createPublisher("set_landing", stdmsg.Bool, 1)


    # Establishes the callbacks for the SwarmLandingSequencer object.  The
    # object subscribes to the swarm_uav_states and topic for own-aircraft,
    # swarm, and subswarm states.
    # @param params: list of required parameters (none for now)
    def callbackSetup(self, params=[]):
        self.createSubscriber("swarm_uav_states", apmsg.SwarmStateStamped, \
                              self._process_swarm_uav_states)
        self.createSubscriber("update_subswarm", stdmsg.UInt8, \
                              self._process_upate_subswarm)
        self.createSubscriber("status", apbrgmsg.Status, \
                              self._process_autopilot_status)


    # Establishes the services for the SwarmLandingSequencer object.
    # @param params: list of required parameters (none for now)
    def serviceSetup(self, params=[]):
        self.createService('init_landing_sequencer', apsrv.SetInteger, \
                           self._init_landing_sequencer)


    # Establishes the services proxies for the SwarmLandingSequencer object.
    # @param params: list of required parameters (none for now)
    def serviceProxySetup(self, params=[]):
        self._wp_getrange_srv_proxy = \
            self.createServiceProxy('wp_getrange', apbrgsrv.WPGetRange)
        self._fpr_param_set_srv_proxy = \
            self.createServiceProxy('fpr_param_set', apbrgsrv.ParamSet)


    # Runs one iteration of the controller, to include processing
    # object-specific data and publishing required control messages
    def runController(self):
        with self._lock:

            # Make sure the aircraft this one is following is still valid
            # If it is not, need to figure out a new aircraft to follow
            if ((self._leaderID not in self._subswarm_keys) and \
                (self._controller_state != NEGOTIATE_ORDER) and \
                (self._controller_state != ON_FINAL) and \
                (self._swarm_uav_states[self._leaderID].swarm_state != \
                                                      enums.LANDING)):
                    self._negotiate_landing_order()

            # Newly activated controller--determine landing sequence first
            if self._controller_state == NEGOTIATE_ORDER:
                self._negotiate_landing_order()
                self._controller_state = START_TRANSIT

            # Sequence computed, but need to send the transit waypoint
            elif self._controller_state == START_TRANSIT:
                self.publishWaypoint(self._wp_msg)
                self._controller_state = IN_TRANSIT
                #tell the autopilot we're about to land
                self._ldg_start_publisher.publish(True)
                self.log_dbug("transit to landing stack location initiated")
                return

            # Transiting to the landing stack location
            elif self._controller_state == IN_TRANSIT:
                state = self._swarm_uav_states[self._ownID]
                d = gps.gps_distance(state.state.pose.pose.position.lat, \
                                    state.state.pose.pose.position.lon, \
                                    self._wp_msg.lat, self._wp_msg.lon)
                if gps.gps_distance(state.state.pose.pose.position.lat, \
                                    state.state.pose.pose.position.lon, \
                                    self._wp_msg.lat, self._wp_msg.lon) < LAND_WP_DIST:
                    self.log_dbug("transit to landing stack location complete")
                    self._controller_state = STACK_ARRIVAL
                    self._stack_time = rospy.Time.now()
                else:
                    return  # Not there yet

            elif self._controller_state == STACK_ARRIVAL:
                if (rospy.Time.now() - self._stack_time) > STACK_LOITER_TIME:
                    self._controller_state = IN_STACK

            # In the landing stack--descend after the leader to block 0, then land
            elif self._controller_state == IN_STACK:
                ldrState = self._swarm_uav_states[self._leaderID]
                if ldrState.swarm_state == enums.LANDING:
                    self._leaderID = self._ownID

                # It's this aircraft's turn to land
                if self._leaderID == self._ownID:
                    ldg_wp_cmd = stdmsg.UInt16()
                    ldg_wp_cmd.data = self._ldgWptIndex
                    #make dang sure the autopilot knows we're landing
                    self._ldg_start_publisher.publish(True)
                    self._ldg_wp_publisher.publish(ldg_wp_cmd)
                    self._controller_state = ON_FINAL
                    self.log_dbug("landing waypoint ordered")
                    return

                # Determine what altitude block we need to be in
                block = int((ldrState.state.pose.pose.position.rel_alt - enums.BASE_REL_ALT) + \
                            (enums.ALT_BLOCK_SIZE - ALT_BLOCK_CAPTURE)) / int(enums.ALT_BLOCK_SIZE)
                block = max(0, block + 1)    # Our block is leaderBlock+1
                if block != self._alt_block: # New order only rqd on change
                    self._wp_msg.alt = enums.BASE_REL_ALT + enums.ALT_BLOCK_SIZE * block
                    self.log_dbug("ordered to altitude block %d (%f meters)" %(self._alt_block, self._wp_msg.alt))
                    self._alt_block = block
                    self.publishWaypoint(self._wp_msg)

            # If landing has already been ordered, we're done
            elif self._controller_state == ON_FINAL:
                return

            # Invalid state--should never get here
            else:
                self._log_warn("invalid controller state: %d" %self._controller_state)
                self._controller_state = None
                self.set_ready_state(False)


    #------------------------------------
    # Callbacks for ROS topic subscribers
    #------------------------------------

    # Handle incoming swarm_uav_states messages
    # @param swarmMsg: message containing swarm data (SwarmStateStamped)
    def _process_swarm_uav_states(self, swarmMsg):
        with self._lock:
            self._swarm_uav_states.clear()
            del self._swarm_keys[:]
            del self._subswarm_keys[:]
            for vehicle in swarmMsg.swarm:
                self._swarm_uav_states[vehicle.vehicle_id] = vehicle
                self._swarm_keys.append(vehicle.vehicle_id)
                if vehicle.subswarm_id == self._subswarm_id:
                    self._subswarm_keys.append(vehicle.vehicle_id)


    # Handle incoming update_subswarm messages
    # @param subswarmMsg: message containing the new subswarm assignment
    def _process_upate_subswarm(self, subswarmMsg):
        self._subswarm_id = subswarmMsg.data


    # Handle incoming autopilot status messages (used to get the ID
    # of the currently ordered waypoint so that the rel_alt can be
    # obtained when the controller is initiated)
    # @param statusMsg:  autopilot status message
    def _process_autopilot_status(self, statusMsg):
        self._ordered_wp_id = statusMsg.mis_cur


    #------------------------
    # Object-specific methods
    #------------------------

    # Sets up the swarm sequence landing behavior.  The behavior
    # takes no parameters, so setting it up simply requires the vehicle to 
    # set the initial controller state (NEGOTIATE_ORDER), transit altitude,
    # and altitude block (0).
    # @param srvReq service request message
    # @return Boolean value indicating behavior initiation success or failure
    def _init_landing_sequencer(self, srvReq):
        try:
            ltr_wpt = srvReq.setting
            fence_wpt = None

            wpt = self._wp_getrange_srv_proxy(ltr_wpt, ltr_wpt).points[0]

            if wpt.command != enums.WP_TYPE_LAND_SEQUENCE:
                raise Exception("Invalid land sequence WP type specified")

            while wpt.command != enums.WP_TYPE_LOITER_TO_ALT:
                ltr_wpt += 1
                wpt = self._wp_getrange_srv_proxy(ltr_wpt, ltr_wpt).points[0]
                if wpt.command == enums.WP_TYPE_ENABLE_FENCE:
                    fence_wpt = ltr_wpt

            # At landing, order to the fence enable or loiter-to-alt
            if fence_wpt != None:
                self._ldgWptIndex = fence_wpt
            else:
                self._ldgWptIndex = ltr_wpt

            if wpt.param2 < 0.0:
                self._fpr_param_set_srv_proxy('WP_LOITER_RAD', -BASE_WP_LOITER_RAD)
            else:
                self._fpr_param_set_srv_proxy('WP_LOITER_RAD', BASE_WP_LOITER_RAD)

            last_wpt = self._wp_getrange_srv_proxy(self._ordered_wp_id, \
                                                   self._ordered_wp_id).points[0]

            self._wp_msg.lat = wpt.x
            self._wp_msg.lon = wpt.y
            self._wp_msg.alt = last_wpt.z

            self._alt_block = 0
            self._controller_state = NEGOTIATE_ORDER
            self.set_ready_state(True)
            return True

        except Exception as ex: 
            self.log_warn("Failed to initialize landing sequencer: " + str(ex))
            self.set_ready_state(False)
            return False


    # Computes this UAV's landing position (i.e., which UAV does it follow).
    # When the computation is complete, the controller state is updated to
    # start transit.  Currently, the sole determinant of sequence is altitude
    # when initiated, so the landing order is computed in a single iteration.
    # This method can be modified later to incorporate more deliberative (and
    # possibly negotiated) methods of determining order.
    def _negotiate_landing_order(self):
        low_to_high = []
        with self._lock:
            for uav in self._subswarm_keys:
                low_to_high.append([ self._swarm_uav_states[uav].vehicle_id, \
                                     self._swarm_uav_states[uav].state.pose.pose.position.alt])
        low_to_high = sorted(low_to_high, key = lambda tup: tup[1])

        # Find this UAV's position in the marshal stack and who it follows
        if low_to_high[0][0] == self._ownID:
            self._leaderID = self._ownID   # This UAV is first to land
        else:
            thisUAV = 1
            while low_to_high[thisUAV][0] != self._ownID:
                thisUAV += 1
            self._leaderID = low_to_high[thisUAV - 1][0]

        self.log_dbug("landing sequence UAV %d to follow UAV %d" \
                      %(self._ownID, self._leaderID))


#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    ownAC = int(rospy.get_param("aircraft_id"))

    sequencer = SwarmLandingSequencer("swarm_landing_sequencer", ownAC)

    sequencer.runAsNode(1.0, [], [], [])

