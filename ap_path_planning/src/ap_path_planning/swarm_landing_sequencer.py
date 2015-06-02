#!/usr/bin/env python

# Object that sequences and controls subswarm aircraft for landing.
#
# Duane Davis 2015

# Standard python library imports
import sys

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

LAND_A_WPT = 12  # Index of landing wp sequence "A" start
LAND_B_WPT = 18  # Index of landing wp sequence "B" start

LAND_WP_DIST = 200.0     # Distance from landing wp to start descent
LAND_BASE_ALT = 100.0    # AGL alt order for "next" aircraft to land
ALT_BLOCK_SIZE = 15.0    # Separation between altitude order
ALT_BLOCK_CAPTURE = 3.0  # UAV within lower alt block at this alt offset

# Enumeration for controller states
NEGOTIATE_ORDER = 0
START_TRANSIT = 1
IN_TRANSIT = 2
IN_STACK = 3
LANDING = 4


# Object that sequences and controls subswarm aircraft for landing.
#
# Class member variables:
#   _ownID: this UAV's ID
#   _subswarm_id: subswarm to which this UAV is assigned
#   _leaderID: ID of the UAV to follow in the landing sequence
#   _ldgWptIndex: Index (ID) of the landing waypoint
#   _alt_block: assigned altitude block relative to the base altitude
#   _wp_msg: LLA message object for waypoint orders to be published
#   _swarm_uav_states: dictionary object containing state of all swarm aircraft
#   _swarm_keys: list of all aircraft IDs (keys) in the swarm dictionary
#   _subswarm_keys: list of subswarm aircraft IDs (keys) in the swarm dictionary
#   _ldg_wp_publisher: ROS publisher to direct the UAV to the landing waypoint
#   _wp_getrange_srv_proxy: ROS proxy for using the autopilot/wp_getrange service
#   _controller_state: determines what the controller is currently doing
#   _transit_started: set to True when the transit wpt is issued
#   _transit_complete: set to True when the transit wpt has been reached
#   _landing_ordered: set to True when ordered to the landing wpt
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
        self._leaderID = 0
        self._subswarm_id = 0
        self._ldgWptIndex = LAND_B_WPT
        self._alt_block = 0
        self._wp_msg = apbrgmsg.LLA()
        self._swarm_uav_states = dict()
        self._swarm_keys = []
        self._subswarm_keys = []
        self._wpt_srv_proxy = None
        self._controller_state = None
#        self.DBUG_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Creates a publisher to send the vehicle to the appropriate waypoint ID
    # to initiate landing once it's this aircraft's turn
    # @param params: list or required parameters (none for now)
    def publisherSetup(self, params=[]):
        self._ldg_wp_publisher = \
            self.createPublisher("waypoint_goto", stdmsg.UInt16, 1)


    # Establishes the callbacks for the SwarmLandingSequencer object.  The
    # object subscribes to the swarm_uav_states and topic for own-aircraft,
    # swarm, and subswarm states.
    # @param params: list of required parameters (none for now)
    def callbackSetup(self, params=[]):
        self.createSubscriber("swarm_uav_states", apmsg.SwarmStateStamped, \
                              self._process_swarm_uav_states)
        self.createSubscriber("update_subswarm", stdmsg.UInt8, \
                              self._process_upate_subswarm)


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


    # Runs one iteration of the controller, to include processing
    # object-specific data and publishing required control messages
    def runController(self):

        # Newly activated controller--determine landing sequence first
        if self._controller_state == NEGOTIATE_ORDER:
            self._negotiate_landing_order()

        # Sequence computed, but need to send the transit waypoint
        elif self._controller_state == START_TRANSIT:
            self.publishWaypoint(self._wp_msg)
            self._controller_state = IN_TRANSIT 
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
                self._controller_state = IN_STACK
            else:
                return  # Not there yet

        # In the landing stack--descend after the leader to block 0, then land
        elif self._controller_state == IN_STACK:
            ldrState = self._swarm_uav_states[self._leaderID]
            if ldrState.swarm_state == enums.LANDING:
                self._leaderID = self._ownID

            # It's this aircraft's turn to land
            if self._leaderID == self._ownID:
                ldg_wp_cmd = stdmsg.UInt16()
                ldg_wp_cmd.data = self._ldgWptIndex
                self._ldg_wp_publisher.publish(ldg_wp_cmd)
                self._controller_state = LANDING
                self.log_dbug("landing waypoint ordered")
                return

            # Determine what altitude block we need to be in
            block = int((ldrState.state.pose.pose.position.rel_alt - LAND_BASE_ALT) + \
                        (ALT_BLOCK_SIZE - ALT_BLOCK_CAPTURE)) / int(ALT_BLOCK_SIZE)
            block = max(0, block + 1)    # Our block is leaderBlock+1
            if block != self._alt_block: # New order only rqd on change
                self._wp_msg.alt = LAND_BASE_ALT + ALT_BLOCK_SIZE * block
                self.log_dbug("ordered to altitude block %d (%f meters)" %(self._alt_block, self._wp_msg.alt))
                self._alt_block = block
                self.publishWaypoint(self._wp_msg)

        # If landing has already been ordered, we're done
        elif self._controller_state == LANDING:
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
        # Get the landing waypoint info from the autopilot (hard coded for now)
        wpt = self._wp_getrange_srv_proxy(self._ldgWptIndex, \
                                          self._ldgWptIndex).points[0]
        while wpt.command != enums.WP_TYPE_LOITER_TO_ALT:
            self._ldgWptIndex += 1
            wpt = self._wp_getrange_srv_proxy(self._ldgWptIndex, \
                                              self._ldgWptIndex).points[0]
        self._wp_msg.lat = wpt.x
        self._wp_msg.lon = wpt.y
        self._wp_msg.alt = self._swarm_uav_states[self._ownID].state.pose.pose.position.rel_alt

        self._alt_block = 0
        self._controller_state = NEGOTIATE_ORDER
        self.set_ready_state(True)
        return True


    # Computes this UAV's landing position (i.e., which UAV does it follow).
    # When the computation is complete, the controller state is updated to
    # start transit.  Currently, the sole determinant of sequence is altitude
    # when initiated, so the landing order is computed in a single iteration.
    # This method can be modified later to incorporate more deliberative (and
    # possibly negotiated) methods of determining order.
    def _negotiate_landing_order(self):
        low_to_high = []
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

        self._controller_state = START_TRANSIT
        self.log_dbug("landing sequence UAV %d to follow UAV %d" \
                      %(self._ownID, self._leaderID))


#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    ownAC = int(rospy.get_param("aircraft_id"))

    sequencer = SwarmLandingSequencer("swarm_landing_sequencer", ownAC)

    sequencer.runAsNode(1.0, [], [], [])

