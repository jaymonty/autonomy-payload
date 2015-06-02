#!/usr/bin/env python

#-----------------------------------------------------------------------
# Controller-Selector Node
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

import sys
import threading
import time

import rospy

import std_msgs.msg as std_msgs
import ap_msgs.msg as payload_msgs
import ap_srvs.srv as payload_srvs
import autopilot_bridge.msg as mavbridge_msgs
import autopilot_bridge.srv as mavbridge_srv 
from ap_lib import nodeable
from ap_lib import ap_enumerations as enums

MODE_SWITCH_LOCKOUT_T = 3.0 # Post-mode-switch time to ignore active errors

#-----------------------------------------------------------------------
# Class definitions

# This class encapsulates the interaction with a specific controller.
# It should only be used from within the ControllerSelector class.
#
# Class member variables
#   id: ID (int enumeration) of this particular controller
#   name: name of this particular controller
#   status: most recently received status for this controller
#   status_last: timestamp of the most recently received status update
#   _run_pub: service proxy for calling the controller's activation service
class ControllerType(object):

    # Initializer creates a "minimally" initialized ControllerType object
    # @param c_id:  ID (int enumeration) for the controller
    # @param c_name:  name for the controller
    # @param req_loiter_wp: True if an infinite loiter wp is always used
    def __init__(self, c_id, c_name, req_loiter_wp):
        self.id = c_id
        self.name = c_name
        self.requires_loiter_wp = req_loiter_wp

        self._run_srv = rospy.ServiceProxy('%s/%s_run' %(c_name, c_name), \
                                           payload_srvs.SetBoolean)

        # A dummy message to use until a status message is received
        self.status = payload_msgs.ControllerState()
        self.status.controller_id = c_id
        self.status.sequence = 0
        self.status.is_ready = False
        self.status.is_active = False
        self.status_last = rospy.Time()


    # Called when a new status message is received that matches the ID
    # of this controller.  The method returns the "ready" and "active"
    # statuses for potential use by the calling object
    # @param status message (ap_msgs/ControllerState) being processed
    # @return a tuple (Boolean) with the controller's ready and active states
    def update_status(self, status):
        if status.controller_id != self.id:
            raise Exception("Cannot update status with mismatched id")
        self.status = status
        self.status_last = rospy.Time.now()
        return (self.status.is_ready, self.status.is_active)


    # Activate or deactive this controller type
    # @param active: Boolean value to activate or deactivate the controller
    def set_active(self, active):
        resp = self._run_srv(active)
        self.status.is_active = resp.result
        return resp.result


# This is the class we interface with. It allows adding controller types,
# which means creating objects that encapsulate the interface to controllers,
# and handling incoming messages. It also (should) implement all the state
# checks necessary to ensuring a safe transition into or between controllers.
#
# Class member variables:
#   _controllers: dictionary object (int key) with controller objects
#   _own_lat: current latitude of this vehicle (for safety waypoint)
#   _own_lon: current longitude of this vehicle (for safety waypoint)
#   _own_rel_alt: current relative altitude of this vehicle (for safety waypoint)
#   _loiter_wp_id: ID of loiter waypoint (issue to autopilot before starting a controller)
#   _safety_wp: location (LLA) of the waypoint location to set when leaving payload control
#   _wp_list: list of waypoints from the autopilot
#   _loiter_wps: list of loiter waypoint indices from the autopilot
#   _current_mode: currently active controller (int ID)
#   _ap_status: status object for autopilot data (autopilot_bridge/Status)
#   _ap_status_last: timestamp of the last ap_status update
#   _pub_wpindex: publisher to send vehicle back to the "safe" loiter point
#   _pub_safetywp: publisher to reset the autopilot wp location to the "safe" point
#   _pub_status: publisher to send a status message for the selector
#   _wp_get_all: proxy for wp_getall service calls
#   _wp_getlast: proxy for wp_getlast service calls
#   _lock: prevent unsafe thread interaction between services and callbacks
#   _mode_change_time: time of last control mode change (for error check lockout)
#
# Inherited from Nodeable:
#   nodeName:  Name of the node to start or node in which the object is
#   timer: ROS rate object that controls the timing loop
#   DBUG_PRINT: set true to force screen debug messages (default FALSE)
#   WARN_PRINT: set false to force screen warning messages (default FALSE)
#
# TODO list:
#  1 Improve basename implementation (too hard coded now)
#  7 Implement safety checking in the "loop" method to make sure that an active
#    controller is behaving properly and initiate reset to "safe" mode if it isn't
class ControllerSelector(nodeable.Nodeable):
    ros_basename = 'controllers'
    ros_nodename = 'ctlr_selector'

    # Initializer for the ControllerSelector initializes the object variables,
    # subscribes to the required ROS topics, and registers ROS publishers
    def __init__(self, nodename='ctlr_selector'):
        nodeable.Nodeable.__init__(self, nodename)
        self._loiter_wp_id = None
        self._wp_list = []
        self._loiter_wps = []
        self._safety_wp = mavbridge_msgs.LLA()
        self._lock = threading.RLock()
        self._mode_change_time = None
        self.WARN_PRINT = True

        # Maintain the controller types by ID
        self._controllers = {}
        self._current_mode = 0

        # Want to know autopilot status during state checks
        self._ap_status = None
        self._ap_status_last = rospy.Time()

        self._wp_getall = None
        self._wp_getlast = None
        self._pub_status = None
        self._pub_wpindex = None
        self._pub_safety_wp = None


    #------------------------------------------------
    # Nodeable class "virtual" method implementations
    #------------------------------------------------

    # Initializes ROS services provided by this class.  This class
    # utilizes a "set_selector_mode" service to activate controllers
    # @param params: no parameters required for this method ([])
    def serviceSetup(self, params=[]):
        self.createService('set_selector_mode', payload_srvs.SetInteger, \
                           self._srv_ctlr_mode)


    # Initializes ROS service proxies for this class.  This class requires
    # proxies to call the wp_getall and wp_getlast services to retrieve
    # waypoints from the autopilot
    # @param params: no parameters required for this method ([])
    def serviceProxySetup(self, params=[]):
        self._wp_getall = \
            self.createServiceProxy('wp_getall', mavbridge_srv.WPGetAll)
        self._wp_getlast = \
            self.createServiceProxy('wp_getlast', mavbridge_srv.WPGetAll)


    # Initializes callbacks for ROS topics to which this class subscribes.
    # This class subscribes to the controller state ctlr_status topic, the
    # autopilot status topic, and the autopilot acs_pose topic.
    # @param params: no parameters required for this method ([])
    def callbackSetup(self, params=[]):
        self.createSubscriber("ctlr_status", payload_msgs.ControllerState,
                              self._sub_ctlr_status)
        self.createSubscriber("status", mavbridge_msgs.Status,
                              self._sub_ap_status)
        self.createSubscriber("acs_pose", mavbridge_msgs.Geodometry,
                              self._sub_acs_pose)


    # Virtual method for setting up publishers for topics to which this
    # object will publish information.  Should be overridden by implementing
    # classes that publish to ROS topics (leave alone if no publishers)
    # @param params: no parameters required for this method ([])
    def publisherSetup(self, params=[]):
        self._pub_status = \
            self.createPublisher("selector_status",
                                 payload_msgs.ControllerGroupStateStamped, 1)
        self._pub_wpindex = \
            self.createPublisher("waypoint_goto", std_msgs.UInt16, 1)
        self._pub_safety_wp = \
            self.createPublisher("payload_waypoint", mavbridge_msgs.LLA, 1)


    # Executes one iteration of the object's timed loop
    # Everything is managed by callbacks and services, so the only thing that
    # is required during a timed loop is publication of the status message
    def executeTimedLoop(self):
        self._send_status_message()


    #------------------------
    # Object-specific methods
    #------------------------

    # Add a new controller type by ID and name
    def add_controller(self, c_id, c_name, require_loiter_wp=True):
        if c_id in self._controllers:
            raise Exception("Attempted to redefine controller %u (%s) as %s" % \
                            (c_id, self.controllers[c_id].name, c_name))
        try:
            self._controllers[c_id] = ControllerType(c_id, c_name, require_loiter_wp)
        except Exception as ex:
            raise Exception("Failed to define controller %u (%s)" % \
                            (c_id, c_name))


    # Handle mode change messages.  This method is responsible for making sure
    # all of the requirements are met for activating a controller to include:
    #    The requested controller exists
    #    The autopilot status is current enough to be trusted
    #    The autopilot is in AUTO mode
    #    The controller is initialized and ready (is_ready set to True)
    #
    # If if hasn't been done already, the method directs the autopilot to
    # the predesignated "infinite loiter" waypoint before activating any
    # controller (if none has been designated with the rosparam, the
    # controller will not be able to activate anything).  Before activating
    # a new controller, all other controllers are explicitly deactivated.
    # @param mode: ID (int) of the newly requested active mode
    # @return the ID of the post-method active controller mode
    def _set_ctlr_mode(self, mode):
        try:
            self._lock.acquire()
            stale_time = rospy.Time.now() - rospy.Duration(3)
            
            # Is this reset to NO_PAYLOAD_CONTROL
            if mode == enums.NO_PAYLOAD_CTRL:
                if self._current_mode != enums.NO_PAYLOAD_CTRL:
                    self._deactivate_all_controllers(True)
                return self._current_mode

            # Is requested mode known to us?
            if mode not in self._controllers:
                raise Exception("Unknown controller type")

            # Have we heard from the autopilot recently?
            # NOTE: commented out for now based on autopilot pub rate observed
            #       worst rate--uncomment if required when rate issue resolved
#            if self._ap_status is None or \
#               self._ap_status_last < stale_time:
#                raise Exception("Unknown or stale autopilot status")

            # Is the autopilot in AUTO?
            if self._ap_status.mode != mavbridge_msgs.Status.MODE_AUTO:
                raise Exception("Autopilot not in AUTO")

            # Have we heard from the requested controller recently?
            if self._controllers[mode].status_last < stale_time:
                raise Exception("Unknown or stale controller status")

            # Is the controller ready?
            if not self._controllers[mode].status.is_ready:
                raise Exception("Requested controller is not ready")

            # Is the controller not yet active?
            if self._controllers[mode].status.is_active:
                # NOTE: Not really an error--it's idempotent!
                # We'll report it anyway and then move along (return)
                self.log_dbug("Set Control Mode: Requested controller is already active")
                return self._controllers[mode]

            # If we're not already in payload control, make sure we've got an
            # up-to-date list of infinite loiter waypoints fromn the autopilot
            if self._current_mode == enums.NO_PAYLOAD_CTRL:
                self._retrieve_last_ap_waypoint()
#                self._set_safety_wp()

            # Shut down any other active controllers (don't send to safety waypoint)
            self._deactivate_all_controllers(False)
    
            # Make sure the specified infinite loiter waypoint actually IS one
            if not self._loiter_wp_id in self._loiter_wps:
                raise Exception("Invalid infinite loiter WP specified for controller switch")

            # Make sure the autopilot is using the infinite loiter waypoint
            # TODO: See item 5 in class header (infinite loiter/safe behavior conflict)
            if self._ap_status.mis_cur != self._loiter_wp_id:
                wpindex = std_msgs.UInt16()
                wpindex.data = self._loiter_wp_id
                self._pub_wpindex.publish(wpindex)
                self._lock.release()
                time.sleep(2.0) # delay to wait for the next autopilot update
                # TODO: See item 6 in class header (make sure new controller took)
                self._lock.acquire()

            # Activate controller
            success = self._controllers[mode].set_active(True)

            # Update internal state
            if success:
                self._current_mode = mode
            else:
                self._current_mode = controller.NO_PAYLOAD_CONTROL

        except Exception as ex:
            # If we get here, something is wrong
            # Do the most conservative thing (for now take the payload out of the loop)
            self._deactivate_all_controllers(True)
            self.log_warn("Set Control Mode: " + ex.args[0])

        finally:
            self._mode_change_time = rospy.Time.now()
            self._lock.release()
            return self._current_mode


    # Implementing function for the selector_mode service for requesting
    # activation of a new controller.
    # @param req_msg: service (SetInteger) message with the new control mode
    # @return True if the requested mode was set, or false otherwise
    def _srv_ctlr_mode(self, req_msg):
        new_mode = self._set_ctlr_mode(req_msg.setting)
        if new_mode == req_msg.setting:
            return payload_srvs.SetIntegerResponse(True)
        else:
            return payload_srvs.SetIntegerResponse(False)


    # Callback for the selector_mode topic used to request
    # activating a specific controller
    # @param msg: message (UInt8) containing the newly requested mode
    def _sub_ctlr_mode(self, msg):
        self._set_ctlr_mode(msg.data)


    # Callback for handling controller status messages
    # This method double checks to make sure that the active controller.
    # If the data in this update contradicts the current state (i.e.,
    # self._current_mode controller is inactive or another controller
    # is active) then all controllers will be disabled.  This may or may
    # not be an indication of a problem (e.g., the controller reached a normal
    # end state), but this will make sure that everything here is consistent.
    # The callback allows each controller to report up to 1 erroneous status
    # to account for any potential threading issue (i.e., status message
    # waiting to be processed when the new mode is set).  This will result
    # in a delay of no more than 1 second for an actual controller status error
    # @param msg: controller status message being processed
    def _sub_ctlr_status(self, msg):
        with self._lock:
            if msg.controller_id not in self._controllers:
                self.log_warn("Received status message from unknown controller type")
                raise Exception("Received status message from unknown controller type")
            self._controllers[msg.controller_id].update_status(msg)

            current_time = rospy.Time.now()
            if self._mode_change_time == None:
                self._mode_change_time = current_time

            # ignore state errors for a bit after a mode change
            lockout_check = (current_time - self._mode_change_time).to_sec()
            if lockout_check < MODE_SWITCH_LOCKOUT_T:
                pass
            elif msg.controller_id == self._current_mode and \
                 msg.is_active == False:
                self.log_warn("Activated controller reporting inactive status")
                self._deactivate_all_controllers(True)
            elif msg.is_active == True and \
                 msg.controller_id != self._current_mode:
                self.log_warn("Inactive controller (" + \
                              str(msg.controller_id) + ") reporting active status")
                self._deactivate_all_controllers(True)


    # Callback for handling autopilot status messages
    # If the autopilot switches out of AUTO mode while a controller is enabled,
    # all controllers will be deactivated.  The method allows for 1 erroneous
    # status report to account for a potential threading issue (i.e., an AP
    # status message waiting to be processed when payload control is initiated).
    # @param msg: new autopilot status message
    def _sub_ap_status(self, msg):
        with self._lock:
            self._ap_status = msg
            self._ap_status_last = rospy.Time.now()
            if self._current_mode != 0:
                if self._ap_status.mode != mavbridge_msgs.Status.MODE_AUTO:
                    self.log_warn("autopilot mode (%d) not compatible with waypoint control" \
                                  %self._ap_status.mode)
                    self._deactivate_all_controllers(False) # don't send to safety waypoint
                elif self._ap_status.mis_cur != self._loiter_wp_id and \
                     self._controllers[self._current_mode].requires_loiter_wp:
                    self.log_warn("autopilot waypoint number (%d) not correct" \
                                  %self._ap_status.mis_cur)
                    self._deactivate_all_controllers(False) # don't send to safety waypoint


    # Callback for handling pose messages for this aircraft
    # @param msg: pose message from the autopilot/acs_pose topic
    def _sub_acs_pose(self, msg):
        self._own_lat = msg.pose.pose.position.lat
        self._own_lon = msg.pose.pose.position.lon
        self._own_rel_alt = msg.pose.pose.position.rel_alt


    # Sends a "deactivate" command to every controllers
    # @param sendToSafetyPoint: Boolean--True will send to the safety waypoint location
    def _deactivate_all_controllers(self, sendToSafetyPt):
        for c in self._controllers:
            self._controllers[c].set_active(False)
        self._current_mode = enums.NO_PAYLOAD_CTRL
        self._set_safety_wp()
        if self._loiter_wp_id != None and sendToSafetyPt:
            self._pub_safety_wp.publish(self._safety_wp)


    # Retrieves and processes all waypoints from the autopilot
    # TODO:  make the service name more configurable
    def _retrieve_ap_waypoints(self):
        try:
            # in case the service wasn't available at startup (ordering)
            if self._wp_getall == None:
                self._wp_getall = \
                    self.setupServiceProxy('wp_getall', mavbridge_srv.WPGetAll)

            self._wp_list = self._wp_getall().points

            # Identify all of the available infinite loiter waypoints
            self._loiter_wps = []
            for wp in self._wp_list:
                if wp.command == enums.WP_TYPE_LOITER:
                    self._loiter_wps.append(wp.seq)

        except Exception as ex:
            self._loiter_wps = []
            self.log_warn("Unable to load waypoints: " + ex.args[0])


    # Retrieves and processes the last waypoint from the autopilot.  This
    # waypoint is used by waypoint controllers to effect the desired
    # control mode.  The waypoint must be an infinite loiter waypoint
    def _retrieve_last_ap_waypoint(self):
        try:
            # in case the service wasn't available at startup (ordering)
            if self._wp_getlast == None:
                self._wp_getlast = \
                    self.setupServiceProxy('wp_getlast', mavbridge_srv.WPGetAll)

            self._wp_list = self._wp_getlast().points
            self._loiter_wps = []
            if len(self._wp_list) > 0 and \
               self._wp_list[0].command == enums.WP_TYPE_LOITER:
                self._loiter_wps.append(self._wp_list[0].seq)

        except Exception as ex:
            self._loiter_wps = []
            self.log_warn("Unable to load last waypoint: " + ex.args[0])


    # Sets the values for the safety waypoint
    def _set_safety_wp(self):
#        self._safety_wp.lat = loiter_pt.x
#        self._safety_wp.lon = loiter_pt.y
#        self._safety_wp.alt = loiter_pt.z
        self._safety_wp.lat = self._own_lat
        self._safety_wp.lon = self._own_lon
        self._safety_wp.alt = self._own_rel_alt
        if len(self._wp_list) < 1:
            raise Exception("No waypoints retrieved from autopilot")

        loiter_pt = self._wp_list[-1]
        if loiter_pt.command == enums.WP_TYPE_LOITER:
            self._loiter_wp_id = loiter_pt.seq
        else:
            self._loiter_wp_id = None
            raise Exception("Last mission waypoint not infinite loiter")


    # Publishes a controller selector status message
    def _send_status_message(self):
        status = payload_msgs.ControllerGroupStateStamped()
        status.header.stamp = rospy.Time.now()
        status.state.active_controller = self._current_mode
        for controller in self._controllers:
            status.state.controllers.append(self._controllers[controller].status)

        self._pub_status.publish(status)


#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # Initialize state machine
    ctlrsel = ControllerSelector(ControllerSelector.ros_nodename)
    ctlrsel.add_controller(enums.WP_SEQUENCE_CTRLR, "wp_sequencer")
    ctlrsel.add_controller(enums.FOLLOW_CTRLR, "follower")
    ctlrsel.add_controller(enums.LANDING_SEQUENCE_CTRLR, "swarm_landing_sequencer", False)

    # Start loop
    # TODO: Is this fast enough?
    ctlrsel.runAsNode(1.0)

