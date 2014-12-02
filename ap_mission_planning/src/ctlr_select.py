#!/usr/bin/env python

#-----------------------------------------------------------------------
# Controller-Selector Node
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

import sys

import rospy

import std_msgs.msg as std_msgs
import ap_msgs.msg as payload_msgs
import ap_srvs.srv as payload_srvs
import autopilot_bridge.msg as mavbridge_msgs
import autopilot_bridge.srv as mavbridge_srv 
from ap_lib import controller

INFINITE_LOITER_CMD = 17  # Autopilot command ID for infinite loiter waypoints

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
    # @param c_topic_base: basename for the controllers ROS topics
    def __init__(self, c_id, c_name, c_topic_base):
        self.id = c_id
        self.name = c_name

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
#   _nodename: ROS nodename of this object
#   _basename: ROS basename for topics associated with this node
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
#
# TODO list:
#  1 Improve basename implementation (too hard coded now)
#  6 Initiate a wait before activating a controller to verify that the autopilot
#    acknowledges receipt of the loiter waypoint (next status update).  Needed?
#  7 Implement safety checking in the "loop" method to make sure that an active
#    controller is behaving properly and initiate reset to "safe" mode if it isn't
class ControllerSelector(object):
    ros_basename = 'controllers'
    ros_nodename = 'ctlr_selector'

    # Initializer for the ControllerSelector initializes the object variables,
    # subscribes to the required ROS topics, and registers ROS publishers
    def __init__(self, nodename='ctlr_selector', basename='controllers'):
        self._nodename = nodename
        self._basename = basename
        self._loiter_wp_id = None
        self._safety_wp = mavbridge_msgs.LLA()
        self._loiter_wp_id = None

        # Maintain the controller types by ID
        self._controllers = {}
        self._current_mode = 0

        # Want to know autopilot status during state checks
        self._ap_status = None
        self._ap_status_last = rospy.Time()

        self._pub_status = rospy.Publisher("%s/selector_status" % basename,
                                           payload_msgs.ControllerGroupStateStamped)
        self._pub_wpindex = rospy.Publisher("autopilot/waypoint_goto",
                                            std_msgs.UInt16)
        self._pub_safety_wp = rospy.Publisher("autopilot/payload_waypoint",
                                              mavbridge_msgs.LLA)

        rospy.Service('%s/selector_mode' % self._nodename,
                      payload_srvs.SetInteger, self._srv_ctlr_mode)

        rospy.Subscriber("%s/status" % basename,
                         payload_msgs.ControllerState,
                         self._sub_ctlr_status)
        rospy.Subscriber("%s/selector_mode" % basename,
                         std_msgs.UInt8,
                         self._sub_ctlr_mode)
        rospy.Subscriber("autopilot/status",
                         mavbridge_msgs.Status,
                         self._sub_ap_status)
        rospy.Subscriber("autopilot/acs_pose",
                         mavbridge_msgs.Geodometry,
                         self._sub_acs_pose)


    # Add a new controller type by ID and name
    def add_controller(self, c_id, c_name):
        if c_id in self._controllers:
            raise Exception("Attempted to redefine controller %u (%s) as %s" % \
                            (c_id, self.controllers[c_id].name, c_name))
        try:
            self._controllers[c_id] = ControllerType(c_id, c_name, self._basename)
        except Exception as ex:
            raise Exception("Failed to define controller %u (%s): %s" % \
                            (c_id, c_name, ex.args[0]))


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
        stale_time = rospy.Time.now() - rospy.Duration(3)

        try:
            # Is this reset to NO_PAYLOAD_CONTROL
            if mode == controller.NO_PAYLOAD_CTRL:
                if self._current_mode != controller.NO_PAYLOAD_CTRL:
                    self._deactivate_all_controllers()
                return self._current_mode

            # Is requested mode known to us?
            if mode not in self._controllers:
                raise Exception("Unknown controller type")

            # Have we heard from the autopilot recently?
            if self._ap_status is None or \
               self._ap_status_last < stale_time:
                raise Exception("Unknown or stale autopilot status")

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
                rospy.logwarn("Set Control Mode: Requested controller is already active")
                return self._current_mode

            # If we're not already in payload control, make sure we've got an
            # up-to-date list of infinite loiter waypoints fromn the autopilot
            if self._current_mode == controller.NO_PAYLOAD_CTRL:
                self._retrieve_ap_waypoints()
                self._set_safety_wp()

            # Shut down any other active controllers
            # TODO: See item 4 in class header about "safe" waiting behavior
            self._deactivate_all_controllers()
    
            # Make sure the specified infinite loiter waypoint actually IS one
            if not self._loiter_wp_id in self._loiter_wps:
                raise Exception("Invalid infinite loiter WP specified for controller switch")

            # Make sure the autopilot is using the infinite loiter waypoint
            # TODO: See item 5 in class header (infinite loiter/safe behavior conflict)
            if self._ap_status.mis_cur != self._loiter_wp_id:
                wpindex = std_msgs.UInt16()
                wpindex.data = self._loiter_wp_id
                self._pub_wpindex.publish(wpindex)
                # TODO: See item 6 in class header (make sure new controller took)

            # Activate controller
            success = self._controllers[mode].set_active(True)

            # Update internal state
            if success:
                self._current_mode = mode
            else:
                self._current_mode = controller.NO_PAYLOAD_CONTROL

            return self._current_mode

        except Exception as ex:
            # If we get here, something is wrong
            # Do the most conservative thing (for now take the payload out of the loop)
            self._deactivate_all_controllers()
            rospy.logwarn("Set Control Mode: " + ex.args[0])
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
    # @param msg: controller status message being processed
    def _sub_ctlr_status(self, msg):
        if msg.controller_id not in self._controllers:
            raise Exception("Received status message from unknown controller type")
        self._controllers[msg.controller_id].update_status(msg)
        if msg.controller_id == self._current_mode and \
           msg.is_active == False:
            self._deactivate_all_controllers()
        elif msg.is_active == True and \
             msg.controller_id != self._current_mode:
            self._deactivate_all_controllers()


    # Callback for handling autopilot status messages
    # If the autopilot switches out of AUTO mode while a controller is
    # enabled, all controllers will be deactivated.
    # @param msg: new autopilot status message
    def _sub_ap_status(self, msg):
        self._ap_status = msg
        self._ap_status_last = rospy.Time.now()
        if self._current_mode != 0 and \
           self._ap_status.mode != mavbridge_msgs.Status.MODE_AUTO:
            self._deactivate_all_controllers()


    # Callback for handling pose messages for this aircraft
    # @param msg: pose message from the autopilot/acs_pose topic
    def _sub_acs_pose(self, msg):
        self._own_lat = msg.pose.pose.position.lat
        self._own_lon = msg.pose.pose.position.lon
        self._own_rel_alt = msg.pose.pose.position.rel_alt


    # Sends a "deactivate" command to every controllers
    def _deactivate_all_controllers(self):
        for c in self._controllers:
            self._controllers[c].set_active(False)
        self._current_mode = controller.NO_PAYLOAD_CTRL
        if self._loiter_wp_id != None:
            self._pub_safety_wp.publish(self._safety_wp)


    # Retrieves and processes all waypoints from the autopilot
    # TODO:  make the service name more configurable
    def _retrieve_ap_waypoints(self):
        try:
            wp_getall = rospy.ServiceProxy('autopilot/wp_getall', mavbridge_srv.WPGetAll)
            self._wp_list = wp_getall().wp

            # Identify all of the available infinite loiter waypoints
            self._loiter_wps = []
            for wp in self._wp_list:
                if wp.command == INFINITE_LOITER_CMD:
                    self._loiter_wps.append(wp.seq)

        except Exception as ex:
            self._loiter_wps = []
            rospy.logwarn("Unable to load waypoints: " + ex.args[0])


    # Sets the values for the safety waypoint
    def _set_safety_wp(self):
#        self._safety_wp.lat = loiter_pt.x
#        self._safety_wp.lon = loiter_pt.y
#        self._safety_wp.alt = loiter_pt.z
        self._safety_wp.lat = self._own_lat
        self._safety_wp.lon = self._own_lon
        self._safety_wp.alt = self._own_rel_alt
        loiter_pt = self._wp_list[-1]
        if loiter_pt.command == INFINITE_LOITER_CMD:
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


    # Loop!
    def loop(self, rate):
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            # TODO: See item 7 in class header about safety monitoring

            self._send_status_message()
            r.sleep()


#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # ROS init
    rospy.init_node(ControllerSelector.ros_basename)

    # Initialize state machine
    ctlrsel = ControllerSelector()
    ctlrsel.add_controller(controller.WP_SEQUENCE_CTRLR, "wp_sequencer")
    ctlrsel.add_controller(controller.FOLLOW_CTRLR, "follower")

    # Start loop
    # TODO: Is this fast enough?
    ctlrsel.loop(1.0)

