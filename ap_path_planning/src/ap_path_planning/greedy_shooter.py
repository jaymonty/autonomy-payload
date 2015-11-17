#!/usr/bin/env python

# Hunter behavior directing each UAV to individually track and shoot
# "red" UAVs (closest one targeted first)
#
# Duane Davis 2015

# Standard python imports
import math

# ROS imports
import rospy

# ACS imports
import ap_msgs.msg as apmsg
import autopilot_bridge.msg as apbrg
import ap_lib.ap_enumerations as enums
import ap_lib.bitmapped_bytes as bytes
import ap_lib.gps_utils as gps
import ap_lib.quaternion_math as quat
from ap_lib.behavior import *
from ap_lib.waypoint_behavior import *
import ap_path_planning.path_planning_utils as pputils


class GreedyShooter(WaypointBehavior):
    ''' Greedy shooter swarm behavior
    Used to direct the subswarm UAVs to independently track and shoot
    "red" UAVs.  The closest red UAV is targeted first.

    Class member variables:
      _own_uav_id: ID of this aircraft
      _own_pose: pose data for this UAV
      _behavior_state: current behavior state (hunt, track, or shoot)
      _max_range: maximum range of the weapon
      _fov_width: engagement field of view width (degrees) of the weapon
      _fov_height: engagement field of view height (degrees) of the weapon
      _reds_shot: set of red UAVs that have been engaged
      _target_id: ID of the red UAV being tracked or engaged
      _target_lat: latitude of the targeted UAV
      _target_lon: longitude of the targeted UAV
      _target_alt: altitude (meters MSL) of the targeted UAV
      _wp_calc: intercept waypoint calculator
      _envelope_counter: iterations that a target is in the firing envelope
      _firing_report_number: sequence number of the firing report
      _firing_report_publisher: publishes ROS firing report messages
      _behavior_data_publisher: publishes swarm data messages (network firing reports)

    Inherited from WaypointBehavior
      wp_msg: LLA object containing the target-intercept lat/lon
      _wpPublisher: publisher object for publishing waypoint commands
      _last_wp_id: Index (ID) of the last (infinite loiter) waypoint

    Inherited from Behavior
      behaviorID: identifier (int) for this particular behavior
      _subswarm_id: ID of the subswarm to which this UAV is assigned
      _swarm: container for state info for all swarm UAVs
      _swarm_keys: key values (IDs) of all swarm UAVs
      _reds: container for state info for all red UAVs
      _subswarm_keys: key values (IDs) of all subswarm UAVs
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
      INFO_PRING: set true to force screen info messages (default FALSE)
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

    HUNT_TGT = 0    # Determine which red UAV to target
    TRACK_TGT = 1   # Try to get into shooting position
    SHOOT_TGT = 2   # Take the shot
    NO_TGTS = 3     # Nothing to shoot--disable behavior

    MAX_TIME_LATE = rospy.Duration(3.0) # Max no-report time to still chase
    RQD_IN_ENVELOPE = 3   # The number of target tests in the envelope to shoot

    # Target envelope parameters
    MAX_RANGE = 200.0
    FOV_WIDTH = 40.0   # 40 degree horizontal cutout in front of UAV
    FOV_HEIGHT = 80.0  # 80 degree vertical cutout in front of UAV

    def __init__(self, nodename, own_id, max_range=MAX_RANGE, \
                 fov_width=FOV_WIDTH, fov_height=FOV_HEIGHT):
        ''' Class initializer initializes class variables.
        @param nodename: name of the ROS node in which this object exists
        @param own_id: ID (int) of this aircraft
        @param max_range: max range (meters) of the weapon
        @param fov_width: field of view width (degrees) of the weapon
        @param fov_height: field of view height (degrees) of the weapon
        '''
        WaypointBehavior.__init__(self, nodename, enums.GREEDY_SHOOTER)
        self._max_range = max_range
        self._fov_width = fov_width
        self._fov_height = fov_height
        self._behavior_state = GreedyShooter.HUNT_TGT
        self._reds_shot = set()
        self._envelope_counter = 0
        self._own_uav_id = own_id
        self._shot_time = None
        self._own_pose = None
        self._target_id = 0
        self._target_lat = 0.0
        self._target_lon = 0.0
        self._target_alt = 0.0
        self._wp_calc = \
            pputils.InterceptCalculator(self, self._own_uav_id, self._reds)
        self._wp_calc.max_time_late = GreedyShooter.MAX_TIME_LATE
        self._wp_calc.lookahead = 0.0
        self._firing_report_number = 0
        self._firing_report_publisher = None
        self._behavior_data_publisher = None

#        self.DBUG_PRINT = True
#        self.INFO_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    def publisherSetup(self):
        ''' Sets up required ROS publishers
        The object publishes to the send_swarm_behavior_data topic.
        '''
        self._firing_report_publisher = \
            self.createPublisher("firing_reports", \
                                 apmsg.FiringReportStamped, 1)
        self._behavior_data_publisher = \
            self.createPublisher("send_swarm_behavior_data", \
                                 apmsg.BehaviorParameters, 1)


    def callbackSetup(self):
        ''' Sets up required subscriptions and callbacks
        '''
        self.createSubscriber("acs_pose", apbrg.Geodometry, \
                              self._update_own_pose)


    def set_behavior(self, params):
        ''' Sets behavior parameters based on set service parameters
        This behavior requires no parameters (if it's red and flies, it dies)
        @param params: parameters from the set service request
        @return True if set with valid parameters
        '''
        self.subscribe_to_red_states()
        self._reds_shot.clear()
        self._last_wp_id = int(rospy.get_param("last_mission_wp_id"))
        self.set_ready_state(True)
        self.log_info("initializing greedy shooter")
        return True


    def run_behavior(self):
        ''' Executes one iteration of the behavior
        First state is order determination.  After the order has been determined
        intercept waypoints will be generated for each iteration
        '''
        if self._behavior_state == GreedyShooter.HUNT_TGT:
            self._target_id = self._identify_target()
            if self._target_id == None:
                self._behavior_state = GreedyShooter.NO_TGTS
            else:
                self._behavior_state = GreedyShooter.TRACK_TGT

        elif self._behavior_state == GreedyShooter.TRACK_TGT:
            intercept = self._track_target()
            if intercept:
                if gps.hitable(self._own_pose.pose.pose.position.lat, \
                               self._own_pose.pose.pose.position.lon, 
                               self._own_pose.pose.pose.position.alt, 
                               ( self._own_pose.pose.pose.orientation.x, \
                                 self._own_pose.pose.pose.orientation.y, \
                                 self._own_pose.pose.pose.orientation.z, \
                                 self._own_pose.pose.pose.orientation.w ), \
                               self._max_range, self._fov_width, self._fov_height, \
                               self._target_lat, self._target_lon, \
                               self._target_alt):
                    self._envelope_counter = 1
                    self._behavior_state = GreedyShooter.SHOOT_TGT
                    self.log_dbug("Target UAV in firing envelope")
            else:
                self.log_info("Lost contact with target")
                self._behavior_state = GreedyShooter.HUNT_TGT

        elif self._behavior_state == GreedyShooter.SHOOT_TGT:
            intercept = self._track_target()
            if intercept:
                if gps.hitable(self._own_pose.pose.pose.position.lat, \
                               self._own_pose.pose.pose.position.lon, 
                               self._own_pose.pose.pose.position.alt, 
                               ( self._own_pose.pose.pose.orientation.x, \
                                 self._own_pose.pose.pose.orientation.y, \
                                 self._own_pose.pose.pose.orientation.z, \
                                 self._own_pose.pose.pose.orientation.w ), \
                               self._max_range, self._fov_width, self._fov_height, \
                               self._target_lat, self._target_lon, \
                               self._target_alt):
                    self._envelope_counter += 1
                    self._shot_time = rospy.Time.now()
                    if self._envelope_counter >= GreedyShooter.RQD_IN_ENVELOPE:
                        self._make_firing_reports()
                        self._reds_shot.add(self._target_id)
                        self._behavior_state = GreedyShooter.HUNT_TGT
                else:
                    self.log_dbug("Target UAV out of firing envelope")
                    self._behavior_state = GreedyShooter.TRACK_TGT

            else:
                self.log_info("Lost contact with target")
                self._behavior_state = GreedyShooter.HUNT_TGT

        elif self._behavior_state == GreedyShooter.NO_TGTS:
            self.set_ready_state(False)
            self.log_info("No targets available to greedy shooter--disabling")

        else:   # shouldn't get here, but just in case
            self.set_ready_state(False)
            self.log_warn("Invalid greedy shooter state--disabling")


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
            self.wp_msg.alt = self._ap_intent.z
            self.publishWaypoint(self.wp_msg)

        return super(LinearFormation, self).set_pause(pause)


    def _safety_checks(self):
        ''' Conducts behavior-specific safety checks
        @return True if the behavior passes all safety checks (False otherwise)
        '''
        # Make sure the waypoint we're using is correct
        if self._ap_wp != self._last_wp_id:
            self.log_warn("using incorrect waypoint ID--deactivating")
            return False

        return True

    #--------------------------
    # Behavior-specific methods
    #--------------------------

    def _update_own_pose(self, pose_msg):
        ''' Callback for the autopilot/acs_pose topic
        @param pose_msg: message object (Geodometry) with new pose data
        '''
        self._own_pose = pose_msg
        self._wp_calc.own_pose = pose_msg


    def _identify_target(self):
        ''' Determines which red UAV will be targeted
        Will always be the closest red UAV that has not already been shot
        @return the ID (int) of the UAV to target (None if no targets left)
        '''
        t = rospy.Time.now()
        self._target_id = None
        min_range = 1e6
        with self._reds_lock:
            for uav_id in self._reds:
                uav = self._reds[uav_id]
                if uav_id not in self._reds_shot and \
                   (t - uav.state.header.stamp) < GreedyShooter.MAX_TIME_LATE:
                    d = gps.gps_distance(self._own_pose.pose.pose.position.lat, \
                                         self._own_pose.pose.pose.position.lon, \
                                         uav.state.pose.pose.position.lat, \
                                         uav.state.pose.pose.position.lon)
                    if d < min_range:
                        self._target_id = uav_id
                        min_range = d

        if self._target_id == None:
            self.log_info("no targets identified for greedy shooter")
        else:
            self.log_info("greedy shooter identified red UAV %d as target" \
                          %self._target_id)
            self._wp_calc.set_params(self._target_id, 0.0, 0.0, \
                                     self._ap_intent.z, \
                                     pputils.InterceptCalculator.BASE_ALT_MODE)
        return self._target_id


    def _track_target(self):
        ''' Directs the UAV towards the targeted UAV
        Computed path is to the target current position, not an intercept
        @return True if successful track (False means lost contact)
        '''
        tgt_pose = None
        intercept = None
        with self._reds_lock:
            intercept = \
                self._wp_calc.compute_intercept_waypoint(self.wp_msg)
            tgt_pose = self._wp_calc.leader_state()

        if intercept:
            self.publishWaypoint(intercept)
            self._target_lat = tgt_pose.state.pose.pose.position.lat
            self._target_lon = tgt_pose.state.pose.pose.position.lon
            self._target_alt = tgt_pose.state.pose.pose.position.alt
            return True
        return False


    def _make_firing_reports(self):
        ''' Publishes ROS messages associated with firing
        A FiringReportStamped is published to the firing_reports topic for
        ROS bagging (and any other) purposes and a SwarmBehaviorData message
        is published to the send_swarm_behavior_data topic for network relay
        '''
        (roll, pitch, yaw) = \
            quat.quat_to_euler((self._own_pose.pose.pose.orientation.x, \
                                self._own_pose.pose.pose.orientation.y, \
                                self._own_pose.pose.pose.orientation.z, \
                                self._own_pose.pose.pose.orientation.w))
        report_msg = apmsg.FiringReportStamped()
        report_msg.header.stamp = self._shot_time
        report_msg.report.lat = self._own_pose.pose.pose.position.lat
        report_msg.report.lon = self._own_pose.pose.pose.position.lon
        report_msg.report.alt = self._own_pose.pose.pose.position.alt
        report_msg.report.yaw = yaw
        report_msg.report.target_id = self._target_id
        report_msg.report.target_lat = self._target_lat
        report_msg.report.target_lon = self._target_lon
        report_msg.report.target_alt = self._target_alt
        self._firing_report_publisher.publish(report_msg)

        self._firing_report_number += 1
        parser = bytes.FiringReportParser()
        parser.report_num = self._firing_report_number
        parser.time_secs = self._shot_time.secs
        parser.time_nsecs = self._shot_time.nsecs
        parser.lat = self._own_pose.pose.pose.position.lat
        parser.lon = self._own_pose.pose.pose.position.lon
        parser.alt = self._own_pose.pose.pose.position.alt
        parser.heading = yaw
        parser.target_id = self._target_id
        parser.target_lat = self._target_lat
        parser.target_lon = self._target_lon
        parser.target_alt = self._target_alt
        data = parser.pack()
        net_msg = apmsg.BehaviorParameters()
        net_msg.id = bytes.FIRING_REPORT
        net_msg.params = data
        for rep in range(0, 4): # Hope to get at least one through
            self._behavior_data_publisher.publish(net_msg)

        self.log_info("Greedy shooter fired at UAV %d: (%f, %f, %f)" \
                      %(self._target_id, self._target_lat, self._target_lon, \
                        self._target_alt))


#----------------------------------------------
# Runs a node with the fixed formation behavior
#----------------------------------------------

if __name__ == '__main__':
    ownAC = int(rospy.get_param("aircraft_id"))
    shooter = GreedyShooter("greedy_shooter", ownAC)
    shooter.runAsNode(10.0)

