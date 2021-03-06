#!/usr/bin/env python

# Contains functions and classes of potential use with multiple behaviors
#
# Duane Davis 2015

import rospy
import math

import ap_lib.behavior as behavior
import ap_lib.gps_utils as gps
import autopilot_bridge.msg as brdgmsg


class InterceptCalculator(object):
    ''' Used to generate waypoint commands to follow a designated swarm UAV

    Member functions:
      set_params: set the calculator's parameters
      compute_intercept_waypoint: computes an intercept to ordered station
      leader: returns the ID of the UAV being followed
      params: returns a control parameter tuple (leader_id, distance, angle)

    Member variables:
      _owner: behavior object to which this object belongs
      _swarm: dictionary object of swarm UAV records (ap_msgs/SwarmVehicleState)
      _own_id: ID of this UAV
      own_pose: this UAV's pose (if None, the swarm_pose will be used)
      _lead_uav_id: ID of the UAV that is to be followed
      _follow_distance: distance (meters) to be maintained form the leader
      _follow_angle: angle-off-the-bow (radians) to maintained from the leader
      _alt: desired rel_alt (BASE_ALT_MODE) or altitude offset (ALT_SEP_MODE)
      _alt_mode: enumeration for BASE_ALT_MODE or ALT_SEP_MODE
      _is_set: set to True when the object is parameterized
      l1_distance: distance ahead of intercept to steer towards (pseudo L1 ctrl)
      lookahead: max time to use in computing the intercept point
      max_time_late: max allowed time between leader pose updates
    '''

    # Class-specific enumerations and constants

    BASE_ALT_MODE = 0  # commanded altitude to use absolute rel_alt
    ALT_SEP_MODE = 1   # commanded altitude to be relative to lead UAV

    OVERSHOOT = 250.0     # "ahead" distance to place waypoint to avoid capture
    MAX_LOOKAHEAD = 10.0  # max DR time for intercept point computation
    MAX_TIME_LATE = rospy.Duration(6.0) # Max time late of the leader's state
    L1_DIST_STD = 125.0 # "Standard" L1 distance for implementing class use


    def __init__(self, owner_behavior, own_id, swarm, \
                 l1_distance=L1_DIST_STD, \
                 lookahead=MAX_LOOKAHEAD, \
                 max_time_late=MAX_TIME_LATE):
        ''' Initializes the object
        @param owner_behavior: owning Behavior object
        @param own_id: ID (int) of this UAV
        @param swarm: container (dict) for swarm aircraft records
        @param l1_distance: projected aim point distance forward of intercept
        @param lookahead: maximum time to DR to an intercept point
        @param max_time_late: maximum allowable time between leader updates
        '''
        if not isinstance(owner_behavior, behavior.Behavior):
            raise Exception("InterceptCalculator owner must be Behavior")
        if not isinstance(swarm, dict):
            raise Exception("Swarm object must be a dictionary")

        self._owner = owner_behavior
        self._own_id = own_id
        self.own_pose = None
        self._swarm = swarm
        self._lead_uav_id = 0
        self._follow_distance = 0.0
        self._follow_angle = 0.0
        self._alt = 0.0
        self._alt_mode = 0
        self._is_set = False
        self.l1_distance = l1_distance
        self.lookahead = lookahead
        self.max_time_late = max_time_late


    def set_params(self, ldr_id, distance, angle, alt, alt_mode):
        ''' Set the objects control parameters
        Checks the validity of the intended parameters
        @param ldr_id: ID of the uav to be followed
        @param distance: distance (meters) to maintain from the leader
        @param angle: angle-off-the-bow (radians) to maintain from the leader
        @param alt: commanded altitude offset (ALT_SEP_MODE) or base rel_alt
        @param alt_mode: enumeration indicating desired altitude mode
        @return Boolean describing set success or failure
        '''
        if not ldr_id in self._swarm:
            self._owner.log_warn("Requested lead UAV not present in swarm")
            self._is_set = False
        elif alt_mode != InterceptCalculator.BASE_ALT_MODE and \
             alt_mode != InterceptCalculator.ALT_SEP_MODE:
            self._owner.log_warn("Invalid altitude mode requested")
            self._is_set = False
        else:
            self._lead_uav_id = ldr_id
            self._follow_distance = distance
            self._follow_angle = angle
            self._alt = alt
            self._alt_mode = alt_mode
            self._is_set = True
            self._owner.log_dbug("Parameters set: Leader=%d, distance=%f, angle=%f, alt=%f, alt_mode=%d"\
                                 %(self._lead_uav_id, self._follow_distance, \
                                   self._follow_angle, self._alt, self._alt_mode))
        return self._is_set


    def _pre_check(self):
        ''' Determines whether or not a waypoint can be successfully generated
        @return the record for the lead UAV or None
        '''
        if not self._is_set:
            self._owner.log_warn("Cannot compute waypoint--not set")
            return None
        if self.own_pose == None and self._own_id not in self._swarm:
            self._is_set = None
            self._owner.log_warn("Cannot compute waypoint--own ID not in swarm")
            return None
        if self._own_id in self._swarm:
            self.own_pose = self._swarm[self._own_id].state
        lead_uav = self._swarm[self._lead_uav_id]
        if (rospy.Time.now() - lead_uav.state.header.stamp) > self.max_time_late:
            self._owner.log_warn("Cannot compute waypoint--lead UAV time late")
            return None
        return lead_uav


    def compute_intercept_waypoint(self, wpt=None):
        ''' Computes an intercept waypoint for the UAV to steer to
        @param wpt: waypoint object to use for computed values
        @return waypoint object (LLA) with the intercept waypoint (or None)
        '''
        lead_uav = self._pre_check()
        if not lead_uav:
            return None

        # Grab & compute required own & leader state info
        own_lat = self.own_pose.pose.pose.position.lat
        own_lon = self.own_pose.pose.pose.position.lon
        own_crs = math.atan2(self.own_pose.twist.twist.linear.y, \
                             self.own_pose.twist.twist.linear.x)
        own_spd = math.hypot(self.own_pose.twist.twist.linear.x, \
                             self.own_pose.twist.twist.linear.y)

        lead_lat = lead_uav.state.pose.pose.position.lat
        lead_lon = lead_uav.state.pose.pose.position.lon
        lead_alt = lead_uav.state.pose.pose.position.rel_alt
        lead_crs = math.atan2(lead_uav.state.twist.twist.linear.y, \
                              lead_uav.state.twist.twist.linear.x)
        lead_spd = math.hypot(lead_uav.state.twist.twist.linear.x, \
                              lead_uav.state.twist.twist.linear.y)

        # Compute tgt point & project it forward to intercept + l1_distance
        tgt_lat, tgt_lon = gps.gps_newpos(lead_lat, lead_lon, \
                                          (lead_crs + self._follow_angle), \
                                          self._follow_distance)
        time_to_intercept = 0.0
        if own_spd > 0.1:
            time_to_intercept = \
                gps.gps_distance(own_lat, own_lon, tgt_lat, tgt_lon) / own_spd
        time_to_intercept = min(time_to_intercept, self.lookahead)
        tgt_travel = (lead_spd * time_to_intercept) + self.l1_distance
        tgt_lat, tgt_lon = \
            gps.gps_newpos(tgt_lat, tgt_lon, lead_crs, tgt_travel)

        # compute a wpt along the line from the current posit to the intercept
        if not wpt: wpt = brdgmsg.LLA()
        to_tgt = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)
        wpt.lat, wpt.lon = \
            gps.gps_newpos(own_lat, own_lon, to_tgt, InterceptCalculator.OVERSHOOT)
        if self._alt_mode == InterceptCalculator.BASE_ALT_MODE:
            wpt.alt = self._alt
        else:
            wpt.alt = lead_uav.state.pose.pose.rel_alt + self._alt
        self._owner.log_dbug("intercept waypoint (lat, lon, alt) = (%f, %f, %f)"
                             %(wpt.lat, wpt.lon, wpt.alt))
        return wpt


    def leader(self):
        ''' @return the ID of the UAV that is being followed
        '''
        return self._lead_uav_id


    def leader_state(self):
        ''' @return the pose of the UAV that is being followed
        '''
        if self._lead_uav_id in self._swarm:
            return self._swarm[self._lead_uav_id]
        return None


    def params(self):
        ''' @return (leader_id, distance, angle) tuple
        '''
        return (self._lead_uav_id, \
                self._follow_distance, \
                self._follow_angle)


class PFInterceptCalculator(InterceptCalculator):
    ''' Uses a potential field calculation to follow a designated swarm UAV

    Member functions:
      compute_intercept_waypoint: override of parent class method

    Inherited from InterceptCalculator:
      set_params: set the calculator's parameters
      compute_intercept_waypoint: computes an intercept to ordered station
      leader: returns the ID of the UAV being followed
      params: returns a control parameter tuple (leader_id, distance, angle)

    Member variables:
      _dist_coefficient: potential coefficient for distance fm tgt pt
      _align_coefficient: potential coefficient for alignment with lead course
      _intercept_coefficient: potential coefficient for intercept lead
    '''

    def __init__(self, owner_behavior, own_id, swarm):
        ''' Initializes the object
        @param owner_behavior: owning Behavior object
        @param own_id: ID (int) of this UAV
        @param swarm: container (dict) for swarm aircraft records
        '''
        InterceptCalculator.__init__(self, owner_behavior, own_id, swarm)
        self._dist_coefficient = 2.0
        self._align_coefficient = 1.0
        self._intercept_coefficient = 1.0


    def compute_intercept_waypoint(self, wpt=None):
        ''' Computes an intercept waypoint for the UAV to steer to
        @param wpt: waypoint object to use for computed values
        @return waypoint object (LLA) with the intercept waypoint (or None)
        '''
        lead_uav = self._pre_check()
        if not lead_uav:
            return None

        # Convenience variables for state info
        own_lat = self.own_pose.pose.pose.position.lat
        own_lon = self.own_pose.pose.pose.position.lon
        own_spd = math.hypot(self.own_pose.twist.twist.linear.x, \
                             self.own_pose.twist.twist.linear.y)
        lead_lat = lead_uav.state.pose.pose.position.lat
        lead_lon = lead_uav.state.pose.pose.position.lon
        lead_alt = lead_uav.state.pose.pose.position.rel_alt
        lead_x_dot = lead_uav.state.twist.twist.linear.x
        lead_y_dot = lead_uav.state.twist.twist.linear.y
        lead_crs = math.atan2(lead_y_dot, lead_x_dot)
        lead_spd = math.hypot(lead_y_dot, lead_x_dot)

        # Compute tgt positions and distances for potential calculation
        tgt_lat, tgt_lon = gps.gps_newpos(lead_lat, lead_lon, \
                                          (lead_crs + self._follow_angle), \
                                          self._follow_distance)
        dist = gps.gps_distance(own_lat, own_lon, tgt_lat, tgt_lon)
        angle = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)
        x_dist = dist * math.cos(angle)
        y_dist = dist * math.sin(angle)
        time_to_intercept = 0.0
        if own_spd > 0.1:
            time_to_intercept = \
                gps.gps_distance(own_lat, own_lon, tgt_lat, tgt_lon) / own_spd
        time_to_intercept = min(time_to_intercept, self.lookahead)
        lead_travel = lead_spd * time_to_intercept
        x_travel = lead_travel * lead_x_dot
        y_travel = lead_travel * lead_y_dot

        # Compute potential values and target waypoint
        x_potential = self._dist_coefficient * x_dist + \
                      self._align_coefficient * lead_x_dot/lead_spd + \
                      self._intercept_coefficient * x_travel
        y_potential = self._dist_coefficient * y_dist + \
                      self._align_coefficient * lead_y_dot/lead_spd + \
                      self._intercept_coefficient * y_travel
        if not wpt:  wpt = brdgmsg.LLA()
        wpt.lat, wpt.lon = \
            gps.gps_newpos(own_lat, own_lon, \
                           math.atan2(y_potential, x_potential), \
                           InterceptCalculator.OVERSHOOT)
        if self._alt_mode == InterceptCalculator.BASE_ALT_MODE:
            wpt.alt = self._alt
        else:
            wpt.alt = lead_uav.state.pose.pose.rel_alt + self._alt
        self._owner.log_dbug("intercept waypoint (lat, lon, alt) = (%f, %f, %f)"
                             %(wpt.lat, wpt.lon, wpt.alt))
        return wpt


class PNInterceptCalculator(InterceptCalculator):
    ''' Uses a proportional navigation calculation to intercept a designated UAV

    Member functions:
      compute_intercept_waypoint: override of parent class method

    Inherited from InterceptCalculator:
      set_params: set the calculator's parameters
      compute_intercept_waypoint: computes an intercept to ordered station
      leader: returns the ID of the UAV being followed
      params: returns a control parameter tuple (leader_id, distance, angle)

    Member variables:
      _theta: angle from the pursuer to the target (radians)
      _theta_dot: instantaneous rate of change of theta (rad/sec)
      _a_max: maximum commandable lateral acceleration
      _gain: control gain to be used in the PN calculation
      _bias: control bias to be used in the PN calculation
      _L1: distance ahead of the UAV to set the waypoint
    '''

    def __init__(self, owner_behavior, own_id, swarm, \
                 max_time_late=InterceptCalculator.MAX_TIME_LATE):
        ''' Initializes the object
        @param owner_behavior: owning Behavior object
        @param own_id: ID (int) of this UAV
        @param swarm: container (dict) for swarm aircraft records
        '''
        InterceptCalculator.__init__(self, owner_behavior, own_id, swarm, \
                                     0.0, 0.0, max_time_late)
        self._theta = 0.0
        self._theta_dot = 0.0
        self._a_max = 7.0
        self._gain = 2.5
        self._bias = 0.0
        self._L1 = 150.0


    def compute_intercept_waypoint(self, wpt=None):
        ''' Computes an intercept waypoint for the UAV to steer to
        @param wpt: waypoint object to use for computed values
        @return waypoint object (LLA) with the intercept waypoint (or None)
        '''
        lead_uav = self._pre_check()
        if not lead_uav:
            return None

        # Update pursuer and target position and velocity info
        lead_lat, lead_lon = \
            gps.gps_newpos(lead_uav.state.pose.pose.position.lat, \
                           lead_uav.state.pose.pose.position.lon, \
                           self._follow_distance,  self._follow_angle)
        lead_spd = math.hypot(lead_uav.state.twist.twist.linear.x, \
                              lead_uav.state.twist.twist.linear.y)
        lead_crs = math.atan2(lead_uav.state.twist.twist.linear.y, \
                              lead_uav.state.twist.twist.linear.x)
        own_lat = self.own_pose.pose.pose.position.lat
        own_lon = self.own_pose.pose.pose.position.lon
        own_spd = math.hypot(self.own_pose.twist.twist.linear.x, \
                             self.own_pose.twist.twist.linear.y)
        if (own_spd - lead_spd) < 1.0: own_spd = lead_spd + 1.0
        own_crs = math.atan2(self.own_pose.twist.twist.linear.y, \
                             self.own_pose.twist.twist.linear.x)

        # Compute the control parameters
        self._theta = gps.gps_bearing(own_lat, own_lon, lead_lat, lead_lon)
        tgt_distance = gps.gps_distance(own_lat, own_lon, lead_lat, lead_lon)
        crossing_spd = own_spd * math.sin(self._theta - own_crs) + \
                       lead_spd * math.sin(self._theta + math.pi - lead_crs)
        self._theta_dot = crossing_spd / tgt_distance
        theta_diff = gps.normalize_pi(self._theta - own_crs)
        self._bias = min(gps.signum(theta_diff) * 2.5, theta_diff / 10.0)

        # Use the PN calculation to compute the ordered acceleration
        ordered_a = (self._gain * self._theta_dot + self._bias) * own_spd
        a_max = min(self._a_max, ((2.0 * math.pow(own_spd, 2.0)) / self._L1))
        if math.fabs(ordered_a) > a_max:
            ordered_a = gps.signum(ordered_a) * a_max
        nu = math.asin((ordered_a * self._L1) / (2.0 * math.pow(own_spd, 2.0)))

        # Compute the waypoint command
        if not wpt:  wpt = brdgmsg.LLA()
        wpt.lat, wpt.lon = \
            gps.gps_newpos(lead_lat, lead_lon, nu + own_crs, self._L1)

        if self._alt_mode == InterceptCalculator.BASE_ALT_MODE:
            wpt.alt = self._alt
        else:
            wpt.alt = lead_uav.state.pose.pose.rel_alt + self._alt
        self._owner.log_dbug("intercept waypoint (lat, lon, alt) = (%f, %f, %f)"
                             %(wpt.lat, wpt.lon, wpt.alt))
        return wpt


class WaypointSequencer(object):
    ''' Used to contain and manage an ordered sequence of waypoints
    Each waypoint is maintained as an LLA object containing the
    latitude, longitude, and rel_alt

    Member functions:
        set_sequence: set the sequence of waypoints
        append: add a waypoint to the list
        get_next: gets the next waypoint (or None if at the end)
        get_next_wrap: gets the next waypoint, wraps to waypoint[0] at end

    Member variables:
    _owner: behavior object to which this object belongs
    _swarm: dictionary object of swarm UAV records (ap_msgs/SwarmVehicleState)
    _sequence: list of waypoint objects
    _current: index of the next waypoint to be retrieved
    '''

    def __init__(self, owner_behavior, swarm):
        ''' Initializes the object
        @param owner_behavior: owning Behavior object
        @param swarm: container (dict) for swarm aircraft records
        '''
        if not isinstance(owner_behavior, behavior.Behavior):
            raise Exception("WaypointSequencer owner must be Behavior")

        if not isinstance(swarm, dict):
            raise Exception("Swarm object must be a dictionary")

        self._owner = owner_behavior
        self._swarm = swarm
        self._sequence = []
        self._current = 0


    def set_sequence(self, waypoints):
        ''' Initializes the waypoint sequence
        @param waypoints: list of waypoints of the form (lat, lon, rel_alt)
        '''
        self._sequence = []
        for wpt in waypoints:
            new_wpt = brdgmsg.LLA()
            new_wpt.lat = wpt[0]
            new_wpt.lon = wpt[1]
            new_wpt.alt = wpt[2]
            self._sequence.append(new_wpt)
        self._current = 0


    def append(self, lat, lon, rel_alt):
        ''' Appends a new waypoint to the end of the sequence
        @param lat: latitude of the new waypoint
        @param lon: longitude of the new waypoint
        @param rel_alt: rel_alt of the new waypoint
        '''
        new_wpt = brdgmsg.LLA()
        new_wpt.lat = lat
        new_wpt.lon = lon
        new_wpt.alt = rel_alt
        self._sequence.append(new_wpt)


    def get_next(self):
        ''' Returns the next waypoint in the sequence or None if at the end
        @return: the next waypoint in the sequence (None if empty)
        '''
        if self._current >= len(self._sequence):
            return None
        wpt = self._sequence[self._current]
        self._current += 1
        return wpt


    def get_next_wrap(self):
        ''' Returns the next waypoint in the sequence (wraps to 0 from the end)
        @return the next waypoint in the sequence (None if empty)
        '''
        if self._current == len(self._sequence):
            self._current = 0
        return self.get_next()


