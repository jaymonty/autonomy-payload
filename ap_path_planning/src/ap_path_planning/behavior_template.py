#!/usr/bin/env python

# Provides a template for the implementation of new behaviors
#
# Duane Davis 2015

# Standard Python imports here

# ROS imports
import rospy
# Aditional ROS imports here

# ACS imports
import ap_msgs.msg as apmsg
import autopilot_bridge.msg as brgmsg
import ap_lib.ap_enumerations as enums
import ap_lib.bitmapped_bytes as bytes
from ap_lib.behavior import *
from ap_lib.waypoint_behavior import *
# Additional ACS imports here (see ap_lib and ap_path_planning packages)


class NewBehavior(WaypointBehavior):
    ''' Class template inheriting from the WaypointBehavior class
    These behaviors use a moving waypoint to control the vehicle's behavior.
    Differences between this type of behavior and those inheriting from the
    Behavior class (which can use any other user-defined mechanism to control
    the vehicle) will be identified where applicable.

    Class-specific member variables:
      _own_uav_id: the ID of this particular UAV (optional, but often useful)

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
      INFO_PRINT: set true to force screen info messages (default FALSE)
      WARN_PRINT: set false to force screen warning messages (default FALSE)

    Class member functions:
      publisherSetup: Nodeable virtual method for ROS publisher setup 
      callbackSetup: Nodeable virtual method for ROS subscription setup
      serviceSetup: Nodeable virtual method for ROS service setup 
      serviceProxySetup: Nodeable virtual method for ROS service proxy setup
      set_behavior: sets behavior-specific parameters before execution
      run_behavior: implementation of the Behavior virtual function
      set_pause: implementation of the Behavior virtual function
      pause_behavior: overrides the Behavior class method
      _safety_checks: implementation of the Behavior virtual function
    '''

    def __init__(self, nodename, behavior_id):
        ''' Class initializer initializes class variables.
        @param nodename: name of the ROS node in which this object exists
        @param behavior_id: unique ID (int) of the behavior object (can be
                            omitted here if a fixed value is to be used, but it
                            is rqd in the call to the parent class constructor)
        '''
        WaypointBehavior.__init__(self, nodename, behavior_id)
        self._own_uav_id = rospy.get_param("aircraft_id")  # If required
        # Initialize any other behavior-specific variables
#        self.DBUG_PRINT = True
#        self.INFO_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Setup methods can be implemented as required to initialize any
    # behavior-specific subscriptions, publishers, services and service
    # proxies.  All of these methods are called automatically by the
    # Nodeable class runAsNode method.

    def publisherSetup(self):
        ''' Sets up required ROS publishers
        Use the Nodeable class createPublisher method to enforce standardized
        naming (node_name/topic_name).  Topics can be remapped as required in
        the launch file.
        NOTE: All behaviors have publisher objects for the
              swarm_control/behavior_status (self._statusPublisher) and
              network/send_swarm_ behavior_data (self._behaviorDataPublisher)
              topics by default.  The self._behaviorDataPublisher can be used
              by behavior implementations to send messages.  If no behavior-
              specific publishers are required, this method can be ommitted.
              (ap_msgs.msg.BehaviorParameters)to other swarm UAVs as required.
              Behavior implementations will not typically use the 
              self._statusPublisher object.
        '''
        pass
#        self._publisher = \
#            self.createPublisher("topic_name", msg_type, queue_size, latched_bool)


    def callbackSetup(self):
        ''' Sets up required ROS topic subscriptions (callbacks)
        Use the Nodeable class createSubscriber method to enforce standardized
        naming (node_name/topic_name).  Topics can be remapped as required in
        the launch file.  If no behavior-specific subscriptions are required,
        this method can be ommitted.
        NOTE: All behaviors subscribe to the autopilot/status, 
              swarm_control/subswarm_id, swarm_control/ap_intent, and
              network/recv_swarm_data topics automatically.  Subscription to
              the swarm_tracker/swarm_uav_states and red_tracker/red_uav_states
              topics should be handled dynamically as required in the
              set_behavior method (see below).
        '''
        pass
#        self.createSubscriber("topic_name", msg_type, callback_method)


    def serviceSetup(self):
        ''' Sets up required ROS services provided by this behavior
        Use the Nodeable class createService method to enforce standardized
        naming (node_name/svc_name).  Services can be remapped as required in
        the launch file.  If no behavior-specific services are required, this
        method can be ommitted.
        NOTE:  All behaviors provide set, run, and pause services by default.
               Implementation of the run service is handled by the parent class
               and is not required for implementing classes.  Behavior-specific
               implementation is required for the set service and optional for
               the pause service using virtual methods of the Behavior class
               (see set_behavior and set_pause methods respectively).
        '''
        pass
#        self.createService("svc_name", svc_type, handler_method)


    def serviceProxySetupetup(self):
        ''' Sets up required ROS service proxies
        Use the Nodeable class createServiceProxy method to enforce standardized
        standardized naming (node_name/svc_name).  Topics can be remapped as
        required in the launch file.  If no behavior-specific service proxies
        are required, this method can be ommitted.
        '''
        pass
#        self._proxy = \
#            self.createPublisher("svc_name", svc_type)


    # The following set, run, and safety check methods must be
    # implemented for all behaviors.  They are called automatically at
    # runtime when the behavior is activated and during execution.

    def set_behavior(self, params):
        ''' Sets behavior parameters based on set service parameters
        The params parameter is parsed with a customized ap_lib.BitmappedBytes
        object that is specific to the behavior being implemented.
        @param params: byte array from the set service request
        @return True if set with valid parameters
        '''
        self.subscribe_to_swarm()  # If swarm UAV state is required when active
        self.subscribe_to_reds()   # If red UAV state is required when active
        parser = bytes.BitmappedBytes() # Behavior-specific parser rqd
        parser.unpack(params)
        # Use parser values to set behavior-specific parameters
        self.set_ready_state(True)  # False if requested settings are not valid
        return self.is_ready


    def _safety_checks(self):
        ''' Conducts behavior-specific safety checks
        This method is called automatically every iteration prior to calling
        the _run_behavior method.
        @return True if the behavior passes all safety checks (False otherwise)
        '''
        # Implement any behavior-specific checks here (things that must be true
        # in order for the behavior to be safely executed).  If any safety check
        # fails, the method should return False at that point
        return True  # If it gets here, all safety checks have passed


    def run_behavior(self):
        ''' Executes one iteration of the behavior
        For WaypointBehavior subclasses, an LLA object is provided for
        encode computed waypoint values (NOTE: must use rel_alt!)
        Other types of behaviors implement their control mechanism instead.
        '''
        # Behavior-specific state calculation and waypoint computation
        self.publishWaypoint(self.wp_msg) # if WaypointControl subclass


    def set_pause(self, pause):
        ''' Pauses and unpauses the behavior (no effect on inactive behaviors)
        The default for this method will cause the UAV to orbit at the last
        ordered waypoint location (if it is a WaypointBehavior).  If this
        is not desireable, the parent class method should be overridden.
        @param pause: Boolean determining whether to pause or resume
        @return: the post-execution paused state of the behavior
        '''
        # Behavior-specific pause functionality
        pass


    # Inter-vehicle message processing (SwarmBehaviorData messages)

    def _process_swarm_data_msg(self, dataMsg):
        ''' Processes swarm data messages received over the network
        This method is used to implement behavior-specific processing of
        SwarmBehaviorData messages received over the network (they reach
        this method as ap_msgs.msg.BehaviorParameter messages).  All
        behaviors automatically subscribe to the appropriate topic and
        invoke this method upon message receipt.  Message processing should
        be wrapped in an "if self.is_active:" block so that message processing
        is conducted only by active behaviors.  If no inter-vehicle messages
        are required by the behavior, this method can be omitted.
        @param dataMsg: message containing received behavior data
        '''
        if self.is_active:
            # Behavior-specific message processing
            # Message type is indicated with the dataMsg.id
            # The dataMsg.params field can be parsed with a user-defined
            # ap_lib.BitmappedBytes() object
            pass


    #-------------------------------------------------------------
    # Behavior-specific methods
    # Including subscription callbacks and service implementations
    #-------------------------------------------------------------


#----------------------------------------------------------------------------
# Node initialization block can be included here or placed in a separate file
#----------------------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node("node_name")
    behavior_id = 0  # Set to a unique ID for each behavior
    hertz = 10.0     # Set to the desired control loop rate for the behavior
    behavior = NewBehavior("node_name", behavior_id)
    behavior.runAsNode(hertz)

