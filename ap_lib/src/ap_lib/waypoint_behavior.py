#!/usr/bin/env python

#-------------------------------------------------------------------------
# WaypointBehavior
# Duane Davis, 2014
#
# Defines an abstract class for wrapping ACS behaviors that work by
# computing waypoints into ROS objects that can be incorporated into
# other classes or run as independent nodes
#-------------------------------------------------------------------------

# ROS imports
import rospy
from rospy import rostime

# ACS imports
from ap_lib import nodeable
from ap_lib import behavior
from autopilot_bridge.msg import LLA
import ap_lib.ap_enumerations as enums


class WaypointBehavior(behavior.Behavior):
    ''' Abstract class for wrapping a control-order-issuing ACS ROS object
    Control is implemented through the generation of waypoint commands.
    Instantiated objects will provide a waypoint publisher that publishes
    computed waypoints to the appropriate topic.

    Class member variables:
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
      _uses_wp_control: set to True if the behavior drives by waypoint
      _statusPublisher: publisher object for behavior status
      _statusStamp: timestamp of the last status message publication
      _sequence: sequence number of the next status message

    Inherited from Nodeable:
      nodeName:  Name of the node to start or node in which the object is
      timer: ROS rate object that controls the timing loop
      DBUG_PRINT: set true to force screen debug messages (default FALSE)
      INFO_PRINT: set to true to force screen info messages (default FALSE)
      WARN_PRINT: set false to force screen warning messages (default FALSE)

    Class member functions
      publishWaypoint: "safely" publishes a waypoint position
    '''

    def __init__(self, nodename, ctrlrID):
        ''' Class initializer sets up the publisher for the waypoint topic
        @param nodename: name of the node that the object is contained in
        @param ctrlrID: identifier (int) for this particular behavior
        '''
        behavior.Behavior.__init__(self, nodename, ctrlrID)
        self._uses_wp_control = True
        self._last_wp_id = 0
        self.wp_msg = LLA()
        self._wpPublisher = \
            rospy.Publisher("autopilot/payload_waypoint", LLA, \
                            tcp_nodelay=True, latch=True, queue_size=1)


    #---------------------------------------------------------
    # Class-specific methods implementing class functionality
    #---------------------------------------------------------

    def publishWaypoint(self, wp):
        ''' Publishes a computed waypoint to the autopilot topic.
        This method enforces an altitude check to make sure the published 
        waypoint is not below the minimum or above the maximum safe altitude.
        Additional safety checks can be implemented here as required.
        @param wp: computed waypoint (LLA) (must use rel_alt!)
        '''
        #verify valid LLA order
        if wp.alt >= enums.MIN_REL_ALT and wp.alt <= enums.MAX_REL_ALT and \
           abs(wp.lon) < enums.MAX_ABS_LON and \
           abs(wp.lat) < enums.MAX_ABS_LAT:
            self._wpPublisher.publish(wp)
        else:
            self.set_active(False)
            self.log_warn("Altitude control mode invalid or invalid altitude order")
