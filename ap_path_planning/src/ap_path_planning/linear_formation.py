#!/usr/bin/env python

# Formation behavior directing UAVs in the subswarm to fly in a
# fixed position, straight line formation relative to one another
#
# Duane Davis 2015

# ROS imports
import rospy
import std_msgs.msg as stdmsg

# ACS imports
import autopilot_bridge.msg as brgmsg
import autopilot_bridge.srv as brgsrv
import ap_msgs.msg as apmsg
import ap_lib.ap_enumerations as enums
import ap_lib.bitmapped_bytes as bytes
import ap_lib.distributed_algorithms as dist
from ap_lib.behavior import *
from ap_lib.waypoint_behavior import *
import ap_path_planning.path_planning_utils as pputils


class LinearFormation(WaypointBehavior):
    ''' Linear formation behavior controller
    Used to direct the UAV when its subswarm is directed to fly in a fixed
    position, straight line formation relative to one another

    Class member variables:
      _own_uav_id: ID of this aircraft
      _swarm: dictionary object containing all swarm records
      _subswarm_id: ID of the subswarm to which this UAV belongs
      _is_lead: True if this UAV is the formation lead
      _distance: distance between UAVs in the formation
      _angle: angle-off-the-bow of the formation line
      _stacked: True if all followers take the same position from the lead
      _wpt_calc: calculator for determining formation waypoints
      _sorter: implements a distributed sort algorithm for formation position
      _wp_goto_publisher: directs the UAV to specific waypoint IDs

    Inherited from WaypointBehavior
      _wpPublisher: publisher object for publishing waypoint commands
      wp_msg: LLA object containing waypoint order
      _last_wp_id: Index (ID) of the last (infinite loiter) waypoint

    Inherited from Behavior
      behaviorID: identifier (int) for this particular behavior
      _subswarm_id: ID of the subswarm to which this UAV is assigned
      _swarm: container for state info for all swarm UAVs
      _swarm_keys: key values (IDs) of all swarm UAVs
      _subswarm_keys: key values (IDs) of all subswarm UAVs
      _ap_intent: most recently ordered autopilot waypoint
      _lock: reentrant lock to enforce thread-safe swarm dictionary access
      is_ready: set to True when the behavior has been initialized
      is_active: set to True when the behavior is running 
      is_paused: set to True when an active behavior is paused
      _activate_time: ROS time that the behavior was activated
      _uses_wp_control: set to True if the behavior drives by waypoint
      _statusPublisher: publisher object for behavior status
      _behaviorDataPublisher: publisher object for behavior data (network) msgs
      _statusStamp: timestamp of the last status message publication
      _sequence: sequence number of the next status message

    Inherited from Nodeable:
      nodeName:  Name of the node to start or node in which the object is
      timer: ROS rate object that controls the timing loop
      DBUG_PRINT: set true to force screen debug messages (default FALSE)
      WARN_PRINT: set false to force screen warning messages (default FALSE)

    Class member functions:
      callbackSetup: implementation of the Nodeable virtual function
      run_behavior: implementation of the Behavior virtual function
      set_pause: implementation of the Behavior virtual function
      pause_behavior: overrides the Behavior class method
      _safety_checks: implementation of the Behavior virtual function
      _setup_formation: determines this UAV position in the formation
    '''
    # Class-specific enumerations and constants

    SETUP = 0
    FLY = 1

    def __init__(self, nodename):
        ''' Class initializer initializes class variables.
        @param nodename: name of the ROS node in which this object exists
        '''
        WaypointBehavior.__init__(self, nodename, enums.SWARM_LINEAR_FORMATION)
        self._own_uav_id = rospy.get_param("aircraft_id")
        self._distance = 0.0
        self._angle = 0.0
        self._stacked = False
        self._is_lead = False
        self._behavior_state = LinearFormation.SETUP
        self._wpt_calc = \
            pputils.InterceptCalculator(self, self._own_uav_id, self._swarm)
        self._sorter = dist.LazyConsensusSort(self._subswarm_keys, \
                                              self._crashed_keys, \
                                              self._behaviorDataPublisher, \
                                              self._swarm_lock)
        self._wp_goto_publisher = None
        self._activate_time = None

#        self.DBUG_PRINT = True
#        self.INFO_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    def publisherSetup(self):
        ''' Sets up required ROS publishers
        The object publishes to the subswarm_id, swarm_state, swarm_behavior
        and autopilot/waypoint_goto topics.
        '''
        self._wp_goto_publisher = \
            self.createPublisher("waypoint_goto", stdmsg.UInt16, 1, False)


    def set_behavior(self, params):
        ''' Sets behavior parameters based on set service parameters
        @param params: parameters from the set service request
        @return True if set with valid parameters
        '''
        self.subscribe_to_swarm()
        parser = bytes.LinearFormationOrderParser()
        parser.unpack(params)
        self._distance = parser.distance
        self._angle = parser.angle
        self._stacked = parser.stack_formation
        self._behavior_state = LinearFormation.SETUP
        self._is_lead = False
        self._last_wp_id = int(rospy.get_param("last_mission_wp_id"))
        self._sorter.reset(self._ap_intent.z)
        self.set_ready_state(True)
        self.log_info("initializing line formation behavior: d=%f, angle=%f, stacked=%s"
                      %(self._distance, self._angle, self._stacked))
        return True


    def run_behavior(self):
        ''' Executes one iteration of the behavior
        First state is order determination.  After the order has been determined
        intercept waypoints will be generated for each iteration
        '''
        self._sorter.send_requested()
        if self._behavior_state == LinearFormation.SETUP:
            if not self._setup_formation():  return

            if self._is_lead:
                self._wp_goto_publisher.publish(stdmsg.UInt16(enums.RACETRACK_WP))
                self.log_info("ordering lead aircraft to the racetrack")

            self._behavior_state = LinearFormation.FLY
            
        elif not self._is_lead:
            with self._swarm_lock:
                wpt = self._wpt_calc.compute_intercept_waypoint(self.wp_msg)
            if wpt:
                self.publishWaypoint(wpt)
            else:
                self.set_ready_state(False)
                self.log_warn("Unable to compute intercept waypoint: disabling")


    def set_pause(self, pause):
        ''' Pauses and unpauses the behavior (no effect on inactive behaviors)
        Overrides the Behavior class method (which is called from this method)
        @param pause: Boolean determining whether to pause or resume
        @return: the post-execution paused state of the behavior
        '''
        if not self.is_active: return
        if pause:
            self.wp_msg.lat = \
                self._swarm[self._own_uav_id].state.pose.pose.position.lat
            self.wp_msg.lon = \
                self._swarm[self._own_uav_id].state.pose.pose.position.lon
            self.wp_msg.alt = \
                self._swarm[self._own_uav_id].state.pose.pose.position.rel_alt
            self.publishWaypoint(self.wp_msg)

        return super(LinearFormation, self).set_pause(pause)


    def _process_swarm_data_msg(self, dataMsg):
        ''' Processes swarm data messages received over the network
        @param dataMsg: message containing received behavior data
        '''
        if self.is_active:
            self._sorter.process_message(dataMsg)


    def _safety_checks(self):
        ''' Conducts behavior-specific safety checks
        @return True if the behavior passes all safety checks (False otherwise)
        '''
        if self._behavior_state == LinearFormation.FLY:

            # Checks if this UAV is not the formation lead
            if not self._is_lead:
                ldr_id = self._wpt_calc.leader()
                ldr_rec = self._swarm[ldr_id]

                # Make sure the UAV we're following is in the right mode
                if ldr_rec.swarm_behavior != enums.SWARM_LINEAR_FORMATION and \
                   (rospy.Time.now() - self._activate_time) > \
                       Behavior.MAX_ACTIVATE_T:
                    self.log_warn("leader in wrong behavior--deactivating")
                    return False

                # Make sure the leader isn't suspected of crashing
                if ldr_id in self._crashed_keys:
                    self.log_warn("no update received from leader (crash?)")
                    return false

                # Make sure the UAV we're following is still in same subswarm
                if ldr_rec.subswarm_id != self._subswarm_id:
                    self.log_warn("leader not in subswarm--deactivating")
                    return False

                # Make sure the UAV we're following is in the right swarm state
                if ldr_rec.swarm_state != enums.SWARM_READY:
                    self.log_warn("leader in wrong swarm state--deactivating")
                    return False

                # Make sure we're using the correct waypoint
                if self._ap_wp != self._last_wp_id:
                    self.log_warn("using incorrect waypoint ID--deactivating")
                    return False

        return True

    #--------------------------
    # Behavior-specific methods
    #--------------------------

    def _setup_formation(self):
        ''' Sorts the subswarm and identifies the UAV this one follows
        Initializes the follow waypoint calculator if this UAV is following
        another one or sets _is_lead to True if it is the formation lead.
        @return True if the formation was successfully set up
        '''
        hi_to_lo = self._sorter.decide_sort()
        if not hi_to_lo: return None
        hi_to_lo.reverse()
            
        result = False
        if hi_to_lo[0][0] == self._own_uav_id:
            self.log_info("line formation initialized with this UAV as leader")
            self._is_lead = True
            result = True

        elif self._stacked:
            self._is_lead = False
            if self._wpt_calc.set_params(hi_to_lo[0][0], self._distance, \
                                         self._angle, self._ap_intent.z, \
                                         pputils.InterceptCalculator.BASE_ALT_MODE):
                result = True
                self.log_info("initialized line formation to follow UAV ID: %d"
                              %self._wpt_calc.leader())

        else:
            step = 1
            while hi_to_lo[step][0] != self._own_uav_id:  step += 1
            if self._wpt_calc.set_params(hi_to_lo[step-1][0], self._distance, \
                                         self._angle, self._ap_intent.z, \
                                         pputils.InterceptCalculator.BASE_ALT_MODE):
                result = True
                self.log_info("initialized line formation to follow UAV ID: %d"
                              %self._wpt_calc.leader())

        if not result:
            self.log_warn("unable to initialize line formation behavior")

        self._activate_time = rospy.Time.now()
        return result


#----------------------------------------------
# Runs a node with the fixed formation behavior
#----------------------------------------------

if __name__ == '__main__':
    rospy.init_node("linear_formation")
    follower = LinearFormation("linear_formation")
    follower.runAsNode(10.0)

