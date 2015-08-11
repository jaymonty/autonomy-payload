#!/usr/bin/env python

#-------------------------------------------------------------------------
# Nodeable
# Duane Davis, 2014
#
# Defines an abstract class for wrapping ACS controllers into ROS objects
# that can be incorporated into other classes or run as independent nodes
#-------------------------------------------------------------------------

# ROS imports
import rospy
from rospy import rostime
from ap_lib import nodeable
import ap_msgs.msg as apmsg
import ap_srvs.srv as apsrv


# Global variables (constants)
CTRLR_BASENAME = 'controllers' # Default base name for controller topics


# Abstract object for wrapping a control-order-issuing ACS ROS object 
# that can be contained in an already established node or run as its
# own independent node
# Instantiated objects will provide a ROS service for activation and
# deactivation.  The service is provided as "nodename/nodename_run"
# and uses the ap_srvs/SetBoolean service.  The service takes a Boolean
# argument designating the desired active state of the node.  The
# service's return value will be the new is_active state of the controller.
# NOTE:  The service will NOT activate an unready node
#
# Class member variables:
#   controllerID: identifier (int) for this particular controller
#   is_ready: set to True when the controller has been initialized
#   is_active: set to True when the controller is running 
#   is_paused: set to True when an active controller is paused
#   _statusPublisher: publisher object for controller status
#   _statusStamp: timestamp of the last status message publication
#   _sequence: number of waypoint sequences that have been ordered
#
# Inherited from Nodeable:
#   nodeName:  Name of the node to start or node in which the object is
#   timer: ROS rate object that controls the timing loop
#   DBUG_PRINT: set true to force screen debug messages (default FALSE)
#   WARN_PRINT: set false to force screen warning messages (default FALSE)
#
# Class member functions
#   executeTimedLoop: implementation of the Nodeable class "virtual" function
#   set_active: "safe" controller activation and deactivation
#   set_ready_state: "safe" transitions between controller ready and not ready
#   _sendIntentMessage: publishes the controller's intent message at 1Hz
#   _sendStatusMessage: publishes the controller's periodic status message
#   _activate_srv: handler for the controller's *_run service
#   _pause_srv: handler for the controller's *_pause service
#
# "Virtual" methods for inheriting class implementation
#   runController: implements 1 iteration of the controller's functionality
class Controller(nodeable.Nodeable):

    # Class initializer initializes base class member variables
    # @param nodename:  name of the node that the object is contained in
    # @param ctrlrID: identifier (int) for this particular controller
    def __init__(self, nodename, ctrlrID):
        nodeable.Nodeable.__init__(self, nodename)
        self.controllerID = ctrlrID
        self.is_ready = False
        self.is_active = False
        self.is_paused = False
        self.intent = apmsg.VehicleIntent()
        self.intent.swarm_behavior = ctrlrID
        self._statusStamp = None
        self._sequence = 0
        self._statusPublisher = \
            self.createPublisher("ctlr_status", apmsg.ControllerState, 1)
        self._intentPublisher = \
            self.createPublisher("payload_intent", apmsg.VehicleIntent, 1)
        self.createService("%s_run"%nodename, apsrv.SetBoolean, self._activate_srv)
        self.createService("%s_pause"%nodename, apsrv.SetBoolean, self._pause_srv)


    #-------------------------------------------------------------------
    # Virtual methods of this class (must be implement in child classes)
    #-------------------------------------------------------------------

    # Runs one iteration of the controller, to include processing
    # object-specific data and publishing required control messages
    def runController(self):
        pass


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Runs a timed single loop of the controller.  If (and only if) the
    # controller is ready and active, the object-specific control
    # method will be called.  Status messages will be published regardless
    def executeTimedLoop(self):
        try:
            if self._statusStamp == None: self._statusStamp = rospy.Time.now()

            # Publish a status message at 1-second intervals
            time = rospy.Time.now()
            interval = (time.secs + (time.nsecs / 1e9)) - \
                       (self._statusStamp.secs + (self._statusStamp.nsecs / 1e9))
            if interval >= 1.0:
                self._sendStatusMessage(time)
                self._sendIntentMessage()
            if self.is_ready and self.is_active and not self.is_paused:
                self.runController()
        except Exception as ex:
            self.log_warn("%s, Loop Error: %s" % (self.nodeName, str(ex)))

    #---------------------------------------------------------
    # Class-specific methods implementing class functionality
    #---------------------------------------------------------

    # Activates or deactivates the controller.  Will not activate a controller
    # that is not "ready" or one that is already active
    # @param activate: Boolean value to activate or deactivate the controller
    def set_active(self, activate):
        if activate and not self.is_ready:
            self.is_active = False
            self.log_warn("attempt to activate uninitialized controller")
        elif activate and not self.is_active:
            self.is_active = activate
            self.log_dbug("activation command: " + str(activate))
            self._sendStatusMessage(rospy.Time.now())
        elif activate == False and self.is_active:
            self.is_active = False
            self._sendStatusMessage(rospy.Time.now())
        return self.is_active


    # Implements the service for activating and deactivating the controller
    # @param activate_req: service call (SetBoolean) message
    # @return service response (SetBoolean) message
    def _activate_srv(self, activate_req):
        resp = self.set_active(activate_req.enable)
        return apsrv.SetBooleanResponse(resp)


    # Sets the controller's ready state when new control inputs are received
    # Will also ensure that the controller is deactivated if the ready state
    # Publishes an updated controller status message if the ready state changes
    # is set to false
    # @param ready: Boolean value to set the controller's ready state
    def set_ready_state(self, ready):
        # Only needs to do anything if the ready state is changing
        if ready and self.is_ready == False:
            self.is_ready = True
            self.is_paused = False
            self._sendStatusMessage(rospy.Time.now())
            self.log_dbug("ready state set to 'True'")
        elif ready == False and self.is_ready:
            self.is_ready = False
            self.is_active = False
            self._sendStatusMessage(rospy.Time.now())
            self.log_dbug("ready state and active state set to 'False'")


    # Pauses and unpauses the controller (will not have any effect on a
    # controller that is not active).  A paused controller should be able to
    # to be unpaused without being re-initialized (i.e., the behavior is still
    # active, its control function is just being bypassed).
    # NOTE: Does not do anything except change the value of the is_paused
    #       member variable.  If more is required to achieve correct behavior,
    #       this method should be overridden by the implementing class.  Call
    #       this version at the end of the subclass definition.
    # @param pause: Boolean value to activate or pause or unpause
    def set_pause(self, pause):
        if not self.is_active: return
        if ((pause and not self.is_paused) or \
            (not pause and self.is_paused)):
            self.is_paused = pause
            self.log_dbug("is_paused set to " + str(pause))
            self._sendStatusMessage(rospy.Time.now())
        return self.is_paused


    # Implements the service for pausing and unpausing the controller
    # @param activate_req: service call (SetBoolean) message
    # @return service response (SetBoolean) message
    def _pause_srv(self, pause_req):
        resp = self.set_pause(pause_req.enable)
        return apsrv.SetBooleanResponse(resp)

    # Publishes a VehicleIntent message
    def _sendIntentMessage(self):
        if self.is_ready and self.is_active and not self.is_paused:
            self._intentPublisher.publish(self.intent)

    # Publishes a ControllerState message to the controller status topic
    # @param time: timestamp for the message
    def _sendStatusMessage(self, time):
        self._statusStamp = time
        status = apmsg.ControllerState()
        status.controller_id = self.controllerID
        status.sequence = self._sequence
        status.is_ready = self.is_ready
        status.is_active = self.is_active
        status.is_paused = self.is_paused
        self._statusPublisher.publish(status)

