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
import autopilot_bridge.msg as mavbridge_msgs
from ap_lib import controller

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
class ControllerType(object):

    # Initializer creates a "minimally" initialized ControllerType object
    # @param c_id:  ID (int enumeration) for the controller
    # @param c_name:  name for the controller
    # @param c_topic_base: basename for the controllers ROS topics
    def __init__(self, c_id, c_name, c_topic_base):
        self.id = c_id
        self.name = c_name

        self._run_pub = rospy.Publisher("%s/%s_run" % (c_topic_base, c_name),
                                        std_msgs.Bool)

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
        msg = std_msgs.Bool()
        msg.data = active
        self._run_pub.publish(msg)


# This is the class we interface with. It allows adding controller types,
# which means creating objects that encapsulate the interface to controllers,
# and handling incoming messages. It also (should) implement all the state
# checks necessary to ensuring a safe transition into or between controllers.
#
# Class member variables:
#   _basename: ROS basename for topics associated with this node
#   _controllers: dictionary object (int key) with controller objects
#   _loiter_wp_param: ROS parameter containing the the loiter waypoint number
#   _loiter_wp_id: ID of loiter waypoint (issue to autopilot before starting a controller)
#   _current_mode: currently active controller (int ID)
#   _ap_status: status object for autopilot data (autopilot_bridge/Status)
#   _ap_status_last: timestamp of the last ap_status update
#   _pub_wpindex: publisher to send vehicle back to the "safe" loiter point
#
# TODO list:
#  1 Improve basename implementation (too hard coded now)
#  2 Implement a selector status message (active controller and whatever else we want)
#  3 Use mavbridge waypoint services to get the loiter waypoint ID (last one in the
#    misison file?) instead of a rosparam (potential source of error).  If the loiter
#    point has not been defined in the mission, can we create one and add it? Maybe
#    near the current location at the time of activation?
#  4 Implement a "safe" behavior for relay to the autopilot when all controllers
#    are deactivated (either intentionally, automatically, or because of error).
#    This might take the form of a mission-defined safety loiter point, but it needs
#    more discussion.
#  5 Make sure that the infinite loiter implementation that takes effect when a
#    new controller is activated doesn't conflict with the "safe" behavior of 4
#  6 Initiate a wait before activating a controller to verify that the autopilot
#    acknowledges receipt of the loiter waypoint (next status update).  Needed?
#  7 Implement safety checking in the "loop" method to make sure that an active
#    controller is behaving properly and initiate reset to "safe" mode if it isn't
class ControllerSelector(object):
    ros_basename = 'controllers'

    # Initializer for the ControllerSelector initializes the object variables,
    # subscribes to the required ROS topics, and registers ROS publishers
    def __init__(self,
                 basename='controllers',
                 loiter_wp_param='payload_loiter_wp'):
        self._basename = basename
        self._loiter_wp_param = loiter_wp_param
        self._loiter_wp_id = None
        try:
            self._loiter_wp_id = rospy.get_param(self._loiter_wp_param)
        except Exception as ex:
            # don't do anything here, but try again if a control mode is
            # activated, and throw an exception if the rosparam isn't set
            pass

        # Maintain the controller types by ID
        self._controllers = {}
        self._current_mode = 0

        # Want to know autopilot status during state checks
        self._ap_status = None
        self._ap_status_last = rospy.Time()

       # TODO: define an overall status message (item 2 in class header)
#        self._pub_status = rospy.Publisher("%s/selector_status" % basename,
#                                           createmessagetype)
        self._pub_wpindex = rospy.Publisher("autopilot/waypoint_goto",
                                            std_msgs.UInt16)

        rospy.Subscriber("%s/status" % basename,
                         payload_msgs.ControllerState,
                         self._sub_ctlr_status)
        rospy.Subscriber("%s/selector_mode" % basename,
                         std_msgs.UInt8,
                         self._sub_ctlr_mode)
        rospy.Subscriber("autopilot/status",
                         mavbridge_msgs.Status,
                         self._sub_ap_status)


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
    def _sub_ctlr_mode(self, msg):
        mode = msg.data
        stale_time = rospy.Time.now() - rospy.Duration(3)
        
        try:
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
                return

            # If we were able to get the infinite loiter waypoint at startup,
            # leave it alone.  If not, check the ROS parameter and get it now.
            # If we still can't get it, throw an exception
            # TODO: See item 3 in class header about the loiter waypoint
            if self._loiter_wp_id == None and rospy.has_param(self._loiter_wp_param):
                try:
                    self._loiter_wp_id = rospy.get_param(self._loiter_wp_param)
                except Exception as ex:
                    raise Exception("Error getting infinite waypoint index: %s" % ex.args[0])
            elif self._loiter_wp_id == None:
                raise Exception("Infinite loiter waypoint parameter not defined")
    
            # Shut down any other active controllers
            # TODO: See item 4 in class header about "safe" waiting behavior
            self._deactivate_all_controllers()
    
            # Make sure the autopilot is using the infinite loiter waypoint
            # TODO: See item 5 in class header (infinite loiter/safe behavior conflict)
            if self._ap_status.mode != self._loiter_wp_id:
                wpindex = std_msgs.UInt16()
                wpindex.data = self._loiter_wp_id
                self._pub_wpindex.publish(wpindex)
                # TODO: See item 6 in class header (make sure new controller took)

            # Activate controller
            self._controllers[mode].set_active(True)

            # Update internal state
            self._current_mode = mode

        except Exception as ex:
            # If we get here, something is wrong
            # Do the most conservative thing (for now take the payload out of the loop)
            self._deactivate_all_controllers()
            rospy.logwarn("Set Control Mode: " + ex.args[0])
            # TODO: See item 4 in class header about a "safe" behavior


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


    # Sends a "deactivate" command to every controllers
    # TODO: See item 4 in class header about a "safe" behavior--put it here
    def _deactivate_all_controllers(self):
        for c in self._controllers:
            self._controllers[c].set_active(False)
        self._current_mode = controller.NO_PAYLOAD_CTRL


    # Loop!
    def loop(self, rate):
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            # TODO: See item 7 in class header about safety monitoring

            # TODO: See item 2 in class header about status message development
            # Publish selector status message
#            status = createmessagetype()
#            status.a_field = None
#            self._pub_status.publish(status)

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

