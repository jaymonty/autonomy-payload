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
import ap_lib.bitmapped_bytes as bytes
import ap_lib.nodeable as nodeable
import ap_lib.gps_utils as gps
import ap_lib.waypoint_behavior as wp_behavior
import ap_lib.distributed_algorithms as dist
import autopilot_bridge.srv as apbrgsrv
import autopilot_bridge.msg as apbrgmsg

LAND_WP_DIST = 75.0     # Distance from landing wp to start descent
ALT_BLOCK_CAPTURE = 3.0  # UAV within lower alt block at this alt offset
STACK_LOITER_TIME = rospy.Duration(15.0) # Min time at stack location before descending

# Enumeration for behavior states
NEGOTIATE_ORDER = 0
START_TRANSIT = 1
IN_TRANSIT = 2
STACK_ARRIVAL = 3
IN_STACK = 4
ON_FINAL = 5

BASE_WP_LOITER_RAD = 50.0


class SwarmLandingSequencer(wp_behavior.WaypointBehavior):
    ''' Object that sequences and controls subswarm aircraft for landing.

    Class member variables:
      _ownID: this UAV's ID
      _subswarm_id: subswarm to which this UAV is assigned
      _leader_id: ID of the UAV to follow in the landing sequence
      _ldg_wpt_index: Index (ID) of the landing waypoint
      _alt_block: assigned altitude block relative to the base altitude
      _ldg_wp_publisher: ROS publisher to direct the UAV to the landing waypoint
      _wp_getrange_srv_proxy: ROS proxy for using the autopilot/wp_getrange service
      _fpr_param_set_srv_proxy: ROS proxy for using the autopilot/fpr_param_set service
      _behavior_state: determines what the behavior is currently doing
      _transit_started: set to True when the transit wpt is issued
      _transit_complete: set to True when the transit wpt has been reached
      _stack_time: ROS time of arrival at the landing stack location
      _sorter: implements a distributed sort algorithm for formation position

    Inherited from WaypointBehavior
      wp_msg: LLA message object for waypoint orders to be published
      _wpPublisher: publisher object for computed waypoints to be sent to the autopilot
      _last_wp_id: Index (ID) of the last (infinite loiter) waypoint

    Inherited from Behavior:
      behaviorID: identifier (int) for this particular behavior
      _subswarm_id: ID of the subswarm to which this UAV is assigned
      _swarm: container for state info for all swarm UAVs
      _swarm_keys: key values (IDs) of all swarm UAVs
      _subswarm_keys: key values (IDs) of all subswarm UAVs
      _ap_intent: most recently ordered autopilot waypoint
      _swarm_lock: reentrant lock to enforce thread-safe swarm dictionary access
      is_ready: set to True when the behavior has been initialized
      is_active: set to True when the behavior is running 
      is_paused: set to True when an active behavior is paused
      _uses_wp_control: set to True if the behavior drives by waypoint
      _statusPublisher: publisher object for behavior status
      _behaviorDataPublisher: publisher object for behavior data (network) msgs
      _statusStamp: timestamp of the last status message publication
      _sequence: sequence number of the next status message

    Inherited from Nodeable:
      nodeName:  Name of the node to start or node in which the object is
      timer: ROS rate object that controls the timing loop
      DBUG_PRINT: set true to force screen debug messages (default FALSE)
      INFO_PRINT: set true to force screen info messages (default FALSE)
      WARN_PRINT: set false to force screen warning messages (default FALSE)

    Class member functions:
      publisherSetup: Nodeable class virtual function implementation
      serviceProxySetup: Nodeable class virtual function implementation
      _set_behavior: Behavior class virtual function implementation
      _run_behavior: Behavior class virtual function implementation
      _safety_checks: Behavior class virtual function implementation
      _negotiate_landing_order: determines what order the UAVs will land in
    '''

    def __init__(self, nodename):
        ''' Class initializer initializes class variables.
        @param nodename: name of the ROS node in which this object exists
        '''
        wp_behavior.WaypointBehavior.__init__(self, nodename, \
            enums.SWARM_SEQUENCE_LAND)
        self._ownID = rospy.get_param("aircraft_id")
        self._leader_id = 0
        self._leader_ok = False
        self._ldg_wpt_index = 0
        self._last_wp_id = None
        self._alt_block = 0
        self._ldg_wp_publisher = None
        self._wp_getrange_srv_proxy = None
        self._fpr_param_set_srv_proxy = None
        self._behavior_state = None
        self._stack_time = None
        self._sorter = dist.ConsensusSort(self._subswarm_keys, \
                                          self._crashed_keys, \
                                          self._behaviorDataPublisher, \
                                          self._swarm_lock)

#        self.DBUG_PRINT = True
#        self.INFO_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    def publisherSetup(self):
        ''' Creates a publisher to send the vehicle to the appropriate waypoint ID
        to initiate landing once it's this aircraft's turn
        '''
        self._ldg_wp_publisher = \
            self.createPublisher("waypoint_goto", stdmsg.UInt16, 1)


    def serviceProxySetup(self):
        ''' Establishes service proxies for the SwarmLandingSequencer object
        '''
        self._wp_getrange_srv_proxy = \
            self.createServiceProxy('wp_getrange', apbrgsrv.WPGetRange)
        self._fpr_param_set_srv_proxy = \
            self.createServiceProxy('fpr_param_set', apbrgsrv.ParamSet)


    def _process_swarm_data_msg(self, dataMsg):
        ''' Processes swarm data messages received over the network
        @param dataMsg: message containing received behavior data
        '''
        if self.is_active:
            self._sorter.process_message(dataMsg)


    def set_behavior(self, params):
        ''' Sets behavior parameters based on set service parameters
        @param params: parameters from the set service request
        @return True if set with valid parameters
        '''
        try:
            self.subscribe_to_swarm()
            parser = bytes.LandingOrderParser()
            parser.unpack(params)
            ltr_wpt = parser.landing_wp_id
            fence_wpt = None

            wpt = self._wp_getrange_srv_proxy(ltr_wpt, ltr_wpt).points[0]

            if wpt.command != enums.WP_TYPE_LAND_SEQUENCE:
                raise Exception("Invalid land sequence WP type specified")

            while wpt.command != enums.WP_TYPE_LOITER_TO_ALT:
                ltr_wpt += 1
                wpt = self._wp_getrange_srv_proxy(ltr_wpt, ltr_wpt).points[0]
                if wpt.command == enums.WP_TYPE_ENABLE_FENCE:
                    fence_wpt = ltr_wpt

            # Re-initialize only if not active or if new landing wpt specified
            if not self.is_active or \
               (self._ldg_wpt_index != fence_wpt and \
                self._ldg_wpt_index != ltr_wpt):

                # At landing, order to the fence enable or loiter-to-alt
                if fence_wpt != None:
                    self._ldg_wpt_index = fence_wpt
                else:
                    self._ldg_wpt_index = ltr_wpt

                if wpt.param2 < 0.0:
                    self._fpr_param_set_srv_proxy('WP_LOITER_RAD', \
                                                  -BASE_WP_LOITER_RAD)
                else:
                    self._fpr_param_set_srv_proxy('WP_LOITER_RAD', \
                                                  BASE_WP_LOITER_RAD)

                self.wp_msg.lat = wpt.x
                self.wp_msg.lon = wpt.y
                self.wp_msg.alt = self._ap_intent.z

                self._sorter.reset(self._ap_intent.z)
                self._alt_block = 0
                self._behavior_state = NEGOTIATE_ORDER
                self._leader_ok = False
                self.set_ready_state(True)

            self._last_wp_id = int(rospy.get_param("last_mission_wp_id"))

            return True

        except Exception as ex: 
            self.log_warn("Failed to initialize landing sequencer: " + str(ex))
            self.set_ready_state(False)
            return False


    def run_behavior(self):
        ''' Runs one iteration of the behavior
        '''
        self._sorter.send_requested()
        with self._swarm_lock:

            # Make sure the aircraft this one is following is still valid
            # If it is not, need to figure out a new aircraft to follow
            if (((self._leader_id not in self._subswarm_keys) or \
                 (self._leader_id in self._crashed_keys)) and \
                (self._behavior_state != NEGOTIATE_ORDER) and \
                (self._behavior_state != ON_FINAL) and \
                (self._swarm[self._leader_id].swarm_state != enums.LANDING)):
                self._negotiate_landing_order()

            # Newly activated behavior--determine landing sequence first
            if self._behavior_state == NEGOTIATE_ORDER:
                if self._negotiate_landing_order():
                    self._behavior_state = START_TRANSIT

            # Sequence computed, but need to send the transit waypoint
            elif self._behavior_state == START_TRANSIT:
                self.publishWaypoint(self.wp_msg)
                self._behavior_state = IN_TRANSIT
                self.log_dbug("transit to landing stack location initiated")
                return

            # Transiting to the landing stack location
            elif self._behavior_state == IN_TRANSIT:
                state = self._swarm[self._ownID]
                d = gps.gps_distance(state.state.pose.pose.position.lat, \
                                    state.state.pose.pose.position.lon, \
                                    self.wp_msg.lat, self.wp_msg.lon)
                if gps.gps_distance(state.state.pose.pose.position.lat, \
                                    state.state.pose.pose.position.lon, \
                                    self.wp_msg.lat, self.wp_msg.lon) < LAND_WP_DIST:
                    self.log_dbug("transit to landing stack location complete")
                    self._behavior_state = STACK_ARRIVAL
                    self._stack_time = rospy.Time.now()
                else:
                    return  # Not there yet

            elif self._behavior_state == STACK_ARRIVAL:
                if (rospy.Time.now() - self._stack_time) > STACK_LOITER_TIME:
                    self._behavior_state = IN_STACK

            # In the landing stack--descend after the leader to block 0, then land
            elif self._behavior_state == IN_STACK:
                ldrState = self._swarm[self._leader_id]
                if ldrState.swarm_state == enums.LANDING:
                    self._leader_id = self._ownID

                # It's this aircraft's turn to land
                if self._leader_id == self._ownID:
                    ldg_wp_cmd = stdmsg.UInt16()
                    ldg_wp_cmd.data = self._ldg_wpt_index
                    self._behavior_state = ON_FINAL
                    self._ldg_wp_publisher.publish(ldg_wp_cmd)
                    self.log_info("landing waypoint ordered")
                    return

                # Determine what altitude block we need to be in
                block = int((ldrState.state.pose.pose.position.rel_alt - enums.BASE_REL_ALT) + \
                            (enums.ALT_BLOCK_SIZE - ALT_BLOCK_CAPTURE)) / int(enums.ALT_BLOCK_SIZE)
                block = max(0, block + 1)    # Our block is leaderBlock+1
                if block != self._alt_block: # New order only rqd on change
                    self.wp_msg.alt = enums.BASE_REL_ALT + enums.ALT_BLOCK_SIZE * block
                    self.log_dbug("ordered to altitude block %d (%f meters)"\
                                  %(self._alt_block, self.wp_msg.alt))
                    self._alt_block = block
                    self.publishWaypoint(self.wp_msg)

            # If landing has already been ordered, we're done
            elif self._behavior_state == ON_FINAL:
                return

            # Invalid state--should never get here
            else:
                self._log_warn("invalid behavior state: %d" %self._behavior_state)
                self._behavior_state = None
                self.set_ready_state(False)


    def _safety_checks(self):
        ''' Conducts behavior-specific safety checks
        @return True if the behavior passes all safety checks (False otherwise)
        '''
        lead = None
        if self._behavior_state != NEGOTIATE_ORDER and \
           self._behavior_state != ON_FINAL:
            lead = self._swarm[self._leader_id]
            if lead.swarm_behavior == enums.SWARM_SEQUENCE_LAND:
                self._leader_ok = True

        # Make sure the UAV we're following is in the right mode
        if lead and self._leader_ok and self._behavior_state == IN_STACK and \
           lead.subswarm_id == self._subswarm_id and \
           lead.swarm_behavior != enums.SWARM_SEQUENCE_LAND and \
           lead.swarm_state != enums.LANDING:
            self.log_warn("followed UAV (%d) wrong behavior (%d)--deactivating" \
                          %(self._leader_id, lead.swarm_behavior))
            return False

        # Make sure the UAV we're following is in the right swarm state
        if lead and lead.swarm_state != enums.SWARM_READY and \
           lead.swarm_state != enums.LANDING:
            self.log_warn("followed UAV (%d) in wrong swarm state--deactivating"\
                          %self._leader_id)
            return False

        # Make sure the waypoint we're using is correct
        if self._behavior_state != ON_FINAL and \
           self._ap_wp != self._last_wp_id:
            self.log_warn("using incorrect waypoint ID--deactivating")
            return False

        return True

    #------------------------
    # Object-specific methods
    #------------------------

    def _negotiate_landing_order(self):
        ''' Computes UAV's landing position (i.e., which UAV does it follow)
        When the computation is complete, the behavior state is updated to
        start transit.  Currently, the sole determinant of sequence is altitude
        when initiated, so the landing order is computed in a single iteration.
        This method can be modified later to incorporate more deliberative (and
        possibly negotiated) methods of determining order.
        '''
        lo_to_hi = self._sorter.decide_sort()
        if not lo_to_hi: return False
        self.log_info("determined low to high order: %s"%str(lo_to_hi))

        # Find this UAV's position in the marshal stack and who it follows
        if lo_to_hi[0][0] == self._ownID:
            self._leader_id = self._ownID   # This UAV is first to land
        else:
            thisUAV = 1
            while lo_to_hi[thisUAV][0] != self._ownID:
                thisUAV += 1
            self._leader_id = lo_to_hi[thisUAV - 1][0]

        self.log_info("landing sequence UAV %d to follow UAV %d" \
                      %(self._ownID, self._leader_id))
        return True

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    rospy.init_node("swarm_landing_sequencer")
    sequencer = SwarmLandingSequencer("swarm_landing_sequencer")
    sequencer.runAsNode(1.0)

