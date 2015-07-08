#!/usr/bin/env python

# Object that sequences and controls subswarm aircraft for takeoff and staging.


# Standard python library imports
import sys
import os
import math
import time

# ROS library imports
import rospy
import std_msgs.msg as stdmsg

# ACS imports
import ap_msgs.msg as apmsg
import ap_srvs.srv as apsrv
import ap_lib.ap_enumerations as enums
import ap_lib.nodeable as nodeable
import ap_lib.gps_utils as gps
import ap_lib.controller as controller
import autopilot_bridge.srv as apbrgsrv
import autopilot_bridge.msg as apbrgmsg

LAND_BASE_ALT = 100.0    # AGL alt order for "next" aircraft to land
BASE_WP_LOITER_RAD = 50.0# Loiter radius distance
ALT_BLOCK_SIZE = 15.0    # Separation between altitude order
ALT_BLOCK_CAPTURE = 3.0  # UAV within lower alt block at this alt offset
MAX_ALTITUDE_SLOT = 550  # This is the maximum height an altitude slot given 30 planes

# Enumeration for controller states
INGRESS_FREE   = 0
INGRESS_LOCKED = 1

# Object that sequences and controls subswarm aircraft for takeoff.
#
# Class member variables:
#   _ownID: this UAV's ID
#   _subswarm_id: subswarm to which this UAV is assigned
#   _active_mode: determines how the calculated alt_slot is used. i.e. waypoints and/or rally
#   _swarm_uav_states: swarm info
#   _ingress_keys: List of keys for ingressing planes
#   _noningress_keys: List of keys for all other planes
#   _locked_slots: Dictionary of uavs to locked slots
#   _ingress_wp: LLA waypoint used for calculating when we breach ingress cylinder
#   _alt_slot: AltitudeSlot message used for storing what our slot is
#   _controller_state:
#   _wp_getrange_srv_proxy: used to retreive _ingress_wp
#   _load_wp_srv_proxy: used to load wp files
#   _load_rally_srv_proxy: used to load rally files
#   _param_setlist_srv_proxy: used to set autopilot params
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
#   _process_update_subswarm: callback to update what subswarm we belong to
#   _process_takeoff_activation: callback to update what active_mode we are in
#   _init_takeoff_sequencer: service for starting off takeoff sequencer
#   _update_locked_slots: Magic function inside update uav_states to calculate whats locked out
#   _negotiate: Passes most of work to update_locked_slots and _next_slot
#   _next_slot: Finds the next nth avaliable slot
#   _generate_new_mission: Loads waypoint and rally files to autopilot
#
class SwarmTakeoffSequencer(controller.Controller):

    # Class initializer initializes class variables.
    # This assumes that the object is already running within an initialized
    # ROS node (i.e., the object does not initialize itself as a node).
    # This enables multiple objects to run within a single node if
    # required.  The initializer does some parameter checking (mostly
    # whether or not all of the required parameters are there), but does
    # not check ranges, magnitudes, or signs.
    # @param nodename: name of the ROS node in which this object exists
    # @param own_id: ID (integer) of this aircraftwp_controller.WaypointController
    def __init__(self, nodename, ctlr_id):
        controller.Controller.__init__(self, nodename, \
            enums.TAKEOFF_SEQUENCE_CTRLR)

        self._ownID = rospy.get_param("aircraft_id")
        self._subswarm_id = 0
        self._active_mode = 0
        self._last_time = 0
        self._swarm_uav_states = dict()
        self._ingress_keys = []
        self._noningress_keys = []
        self._locked_slots = {}
        self._ingress_locked_keys = []
        self._ingress_order = 0
        self._ingress_wp = apbrgmsg.LLA()
        self._alt_slot = apmsg.AltitudeSlot()
        self._controller_state = None
        self._wp_getrange_srv_proxy = None
        self._load_wp_srv_proxy = None
        self._load_rally_srv_proxy = None
        self._param_setlist_srv_proxy = None
        self._wp_goto_publisher = None
        self._altitude_slot_publisher = None
#        self.DBUG_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Creates a publisher to send the vehicle to the appropriate waypoint ID
    # to initiate landing once it's this aircraft's turn
    # @param params: list or required parameters (none for now)
    def publisherSetup(self, params=[]):
        self._wp_goto_publisher = \
            self.createPublisher("waypoint_goto", stdmsg.UInt16, 1)
        self._altitude_slot_publisher = \
            self.createPublisher("intended_alt_slot", apmsg.AltitudeSlot, 1)

    # Establishes the callbacks for the SwarmTakeoffSequencer object.  The
    # object subscribes to the swarm_uav_states and topic for own-aircraft,
    # swarm, and subswarm states.
    # @param params: list of required parameters (none for now)
    def callbackSetup(self, params=[]):
        self.createSubscriber("swarm_uav_states", apmsg.SwarmStateStamped, \
                              self._process_swarm_uav_states)
        self.createSubscriber("update_subswarm", stdmsg.UInt8, \
                              self._process_upate_subswarm)
        self.createSubscriber("active_mode", stdmsg.UInt8, \
                              self._process_takeoff_activation)

    # Establishes the services for the SwarmTakeoffSequencer object.
    # @param params: list of required parameters (none for now)
    def serviceSetup(self, params=[]):
        self.createService('init_takeoff_sequencer', apsrv.SetInteger, \
                           self._init_takeoff_sequencer)


    # Establishes the services proxies for the SwarmTakeoffSequencer object.
    # @param params: list of required parameters (none for now)
    def serviceProxySetup(self, params=[]):
        self._wp_getrange_srv_proxy = \
            self.createServiceProxy('wp_getrange', apbrgsrv.WPGetRange)
        self._load_wp_srv_proxy = \
            self.createServiceProxy('load_wp', apbrgsrv.FileLoad)
        self._load_rally_srv_proxy = \
            self.createServiceProxy('load_rally', apbrgsrv.FileLoad)
        self._param_setlist_srv_proxy = \
            self.createServiceProxy('param_setlist', apbrgsrv.ParamSetList)

    # Runs one iteration of the controller, to include processing
    # object-specific data and publishing required control messages.
    def runController(self):
        # Throttle down rate at which we send altitude_slot messages
        # Verified in sitl looking at ros hz that it comes very close to 1 hz
        time_now = rospy.get_time()
        if self._last_time + 1 < time_now:
            self._altitude_slot_publisher.publish(self._alt_slot)
            self._last_time = time_now

        if self._controller_state == INGRESS_FREE:
            self._negotiate()
        elif self._controller_state == INGRESS_LOCKED:
            # NOTE: Do nothing. Eventually we may want to do more deconfliction
            # using an intended altitude here
            pass
        else:
            self.log_warn("Invalid State")

    #------------------------------------
    # Callbacks for ROS topic subscribers
    #------------------------------------

    # Handle incoming swarm_uav_states messages
    # Break uavs into two categories: Ingress and Noningress
    # @param swarmMsg: message containing swarm data (SwarmStateStamped)
    def _process_swarm_uav_states(self, swarmMsg):
        self._swarm_uav_states.clear()
        del self._ingress_keys[:]
        del self._noningress_keys[:]

        for vehicle in swarmMsg.swarm:
            self._swarm_uav_states[vehicle.vehicle_id] = vehicle

            if vehicle.swarm_state == enums.INGRESS:
                self._ingress_keys.append(vehicle.vehicle_id)
            else:
                self._noningress_keys.append(vehicle.vehicle_id)

        self._update_locked_slots()

    # Handle incoming update_subswarm messages
    # @param subswarmMsg: message containing the new subswarm assignment
    def _process_upate_subswarm(self, subswarmMsg):
        self._subswarm_id = subswarmMsg.data

    # Handle incoming takeoff active mode messages
    # 0: Snipped algorithm
    # 1: Set waypoints only
    # 2: Set waypoints and rally
    # @param stdMsg: message containing an uint8_t
    def _process_takeoff_activation(self, stdMsg):
        self._active_mode = stdMsg.data

    #------------------------
    # Object-specific methods
    #------------------------

    # Sets up the swarm sequence takeoff behavior.  The behavior
    # takes one parameters which is the ingress waypoint id number which is
    # used to retreive the INGRESS CYLINDER WP lat lon.
    # @param srvReq service request message
    # @return Boolean value indicating behavior initiation success or failure
    def _init_takeoff_sequencer(self, srv):
        try:
            wpt = self._wp_getrange_srv_proxy(enums.INGRESS_LOITER_WP, \
                                              enums.INGRESS_LOITER_WP).points[0]
            self._ingress_wp.lat = wpt.x
            self._ingress_wp.lon = wpt.y
            self._alt_slot.altitude_slot = 0
            self._controller_state = INGRESS_FREE
            self.set_ready_state(True)
            return True
        except Exception as ex:
            self.log_warn("Failed to initialize takeoff sequencer: " + str(ex))
            self.set_ready_state(False)
            return False


    # Negotiates where in the stack this plane should reside.
    # First iteration publishes a waypoint goto to the second waypoint
    # which is a loiter for time waypoint. It then finds the next available slot
    # and decides if its within 60m of the INGRESS_LOITER_WP. If it is then it should
    # change its mission files and go to the 3rd waypoint which is a loiter to alt
    def _negotiate(self):
        if self._alt_slot.altitude_slot == 0:
            self._wp_goto_publisher.publish(enums.INGRESS_LOITER_WP)

        self._alt_slot.altitude_slot = self._next_slot(self._ingress_order)

        own_ac = self._swarm_uav_states[self._ownID]
        distance = gps.gps_distance(own_ac.state.pose.pose.position.lat, \
                                    own_ac.state.pose.pose.position.lon, \
                                    self._ingress_wp.lat, \
                                    self._ingress_wp.lon)

        if distance < BASE_WP_LOITER_RAD + 10:
            self._controller_state = INGRESS_LOCKED
            if self._active_mode == 1 or self._active_mode == 2:
                # Testing warning
                # self.log_warn("Takeoff: Altitude slot set as %d" % self._alt_slot.altitude_slot)
                self._generate_new_mission(self._alt_slot.altitude_slot, 1)
                # NOTE: This may be causing an issue. It seems like generate new mission
                # needs to sleep for awhile before it sends a goto waypoint
                self._wp_goto_publisher.publish(enums.INGRESS_CYLINDER_WP)

    # Maintains the dictionary of locked altitude slots. Locked altitudes come
    # from two sources. First any plane that is not in ingress mode will lock out
    # an altitude. Second comes from any plane in ingress that has breached the
    # ingress cylinder.
    def _update_locked_slots(self):
        altitude_slot = 0

        # Update dictionary of locked altitude slots from planes not in ingress
        for uav in self._noningress_keys:
            altitude_slot = LAND_BASE_ALT
            plane = self._swarm_uav_states[uav]
            # Iterate from minumum altitude slot to max altitude slot
            # Lock out the slot for which each plane is closest too
            while altitude_slot <= MAX_ALTITUDE_SLOT:
                if math.fabs(plane.state.pose.pose.position.rel_alt - altitude_slot) < ALT_BLOCK_CAPTURE:
                    self._locked_slots[str(uav)] = [altitude_slot, plane.state.header.stamp.secs]
                    break
                else:
                    altitude_slot += ALT_BLOCK_SIZE

        # Need to sort the planes by altitude so that the ingress_order
        # is correctly calculated which is important for determing
        # other planes altitude slots.
        self._ingress_order = 0
        low_to_high = []
        for uav in self._ingress_keys:
            low_to_high.append([uav, self._swarm_uav_states[uav].state.pose.pose.position.rel_alt])
        low_to_high = sorted(low_to_high, key = lambda low_to_high: low_to_high[1], reverse=True)

        for ndx in range(len(low_to_high)):
            uav = low_to_high[ndx][0]
            # Any plane above me takes away one more free slot that is not in ingress locked
            if low_to_high[ndx][1] > self._swarm_uav_states[self._ownID].state.pose.pose.position.rel_alt \
                and uav not in self._ingress_locked_keys:

                self._ingress_order += 1
                self.log_warn

                distance = gps.gps_distance(self._swarm_uav_states[uav].state.pose.pose.position.lat, \
                                            self._swarm_uav_states[uav].state.pose.pose.position.lon, \
                                            self._ingress_wp.lat, \
                                            self._ingress_wp.lon)

                if distance <= BASE_WP_LOITER_RAD + 10:
                    # Now that this plane is in ingress locked its in dict of locked
                    # altitudes so remove from order.
                    self._ingress_order -= 1
                    # Calculate where it belongs in stack
                    uav_slot = self._next_slot(self._ingress_order)
                    # Testing warning
                    # self.log_warn("My Alt: %d UAV %d at altitude %d going to ingress locked with slot %d" % \
                    #     (self._swarm_uav_states[self._ownID].state.pose.pose.position.rel_alt, uav, self._swarm_uav_states[uav].state.pose.pose.position.rel_alt, uav_slot))
                    self._ingress_locked_keys.append(uav)
                    self._locked_slots[str(uav)] = [uav_slot, self._swarm_uav_states[uav].state.header.stamp.secs]

        # TODO: Reconcile any slots that have more than one occupant
        # Right now we are just going to log the data
        # for idx in self._locked_slots:
        #     for ndx in self._locked_slots:
        #         if idx != ndx and self._locked_slots[idx][0] == self._locked_slots[ndx][0]:
        #             self.log_warn("Takeoff Error: Altitude Confliction at %d" % self._locked_slots[idx][0])


    # This function uses the locked slots dictionary and the number of planes above
    # itself in order to determine the next free slot that it should take
    def _next_slot(self, count):
        alt = LAND_BASE_ALT - ALT_BLOCK_SIZE
        # Iterate until our count is below 0
        # NOTE: getting an altitude greater then max alt should result in an err
        while alt <= MAX_ALTITUDE_SLOT and count >= 0:
            slot_filled = False
            alt += ALT_BLOCK_SIZE

            # Iterate through locked slots and see if curr alt is being used
            # if it is then break, increase alt and restart.
            for uav in self._locked_slots:
                if alt == self._locked_slots[uav][0]:
                    slot_filled = True
                    break

            # If its not filled reduce count meaning were one slot closer to the
            # one we should occupy
            if not slot_filled:
                count -= 1

        return alt

    # Unabashedly copied from Mikes net_mission_config
    # Generates a new WP and Rally mission file from the blessed template files.
    # @param std_alt Altitude to set all wp altitude heights to in mission file
    # @param stack_num The stack in which
    def _generate_new_mission(self, std_alt, stack_num):
        mis_files = []

        # Select whether we are editing both wp and rally or just wp
        mis_files.append('wp')
        if self._active_mode == 2:
            mis_files.append('rally')

        for cfg in mis_files:
            rospy.set_param('ok_' + cfg, False)

        selections = {}
        replacements = {}
        replacements['STDALT'] = int(std_alt)
        selections['STACK'] = stack_num

        if self._active_mode == 2:
            rospy.set_param('ok_param' + cfg, False)
            # Set parameters
            plist = []
            plist.append(apbrgmsg.ParamPair('ALT_HOLD_RTL',
                                             replacements['STDALT'] * 100.0))
            plist.append(apbrgmsg.ParamPair('FENCE_RETALT',
                                             replacements['STDALT']))
            res = self._param_setlist_srv_proxy(plist)
            rospy.set_param('ok_param', res.ok)

        # Build out templates and load them
        for cfg in mis_files:
            # Set up locations
            base_file = os.path.expanduser("~/blessed/%s.template" % cfg)
            temp_file = base_file + "."+str(self._ownID)+".tmp"

            # Modify blessed file into temp location
            with open(base_file, 'r') as inf, open(temp_file, 'w') as outf:
                for line in inf:
                    if not line or not isinstance(line, str):
                        continue

                    # Line selection
                    # NOTE: Can only have one selection pattern per line
                    select = ''
                    for pattern, value in selections.iteritems():
                        if not line.startswith(pattern):
                            continue
                        select = "%s_%s " % (pattern, str(value))
                        break
                    if select:
                        # If wrong selection, omit the line
                        if not line.startswith(select):
                            continue
                        # If right selection, strip the string and use it
                        line = line.replace(select, '')

                    # String replacement
                    # NOTE: May be multiple replacements per line
                    for pattern, value in replacements.iteritems():
                        line = line.replace(str(pattern), str(value))

                    outf.write(line)

            # Call load service
            if cfg == 'wp':
                res = self._load_wp_srv_proxy(temp_file)
            elif cfg == 'rally':
                res = self._load_rally_srv_proxy(temp_file)

            # Update Ok flag
            rospy.set_param('ok_' + cfg, res.ok)
            os.remove(temp_file)

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    ownAC = int(rospy.get_param("aircraft_id"))
    sequencer = SwarmTakeoffSequencer("swarm_takeoff_sequencer", ownAC)
    sequencer.runAsNode(10.0, [], [], [])

        # NOTE: This may need to live in its own function
        # Forced to keep list of keys to delete since you cant
        # delete from a dictionary while iterating over it
        # time = rospy.get_time()
        # uavs_to_delete = []
        # for uav in self._locked_slots:
        #     if uav time > self._locked_slots[uav][1] + 5:
        #         uavs_to_delete.append(uav)
        #
        # for uav in uavs_to_delete:
        #     del self._locked_slots[uav]
