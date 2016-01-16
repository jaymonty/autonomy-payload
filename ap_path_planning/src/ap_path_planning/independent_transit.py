#!/usr/bin/env python

# Simple behavior for all UAVs in the subswarm to transit
# independently to a geographic position
#
# Duane Davis 2016

# ROS imports
import rospy

# ACS imports
import ap_msgs.msg as apmsg
import ap_lib.ap_enumerations as enums
import ap_lib.bitmapped_bytes as bytes
import ap_lib.behavior as behavior
import ap_lib.waypoint_behavior as wpt_behavior


class IndependentTransit(wpt_behavior.WaypointBehavior):
    ''' Simple behavior that sends the vehicle to a geographic position.

    Class member variables:
      _wp_sent: set to True if the waypoint has been sent to the autopilot

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
      run_behavior: implementation of the Behavior virtual function
      pause_behavior: overrides the Behavior class method
      _setup_formation: determines this UAV position in the formation
    '''

    def __init__(self, nodename):
        ''' Class initializer initializes class variables.
        @param nodename: name of the ROS node in which this object exists
        '''
        wpt_behavior.WaypointBehavior.__init__(self, nodename, enums.INDEPENDENT_TRANSIT)
        self._own_uav_id = rospy.get_param("aircraft_id")
        self._wp_sent = False
#        self.DBUG_PRINT = True
#        self.INFO_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    def set_behavior(self, params):
        ''' Sets behavior parameters based on set service parameters
        @param params: parameters from the set service request
        @return True if set with valid parameters
        '''
        sorter = bytes.LatitudeLongitudeParser()
        sorter.unpack(params)
        self.wp_msg.lat = sorter.latitude
        self.wp_msg.lon = sorter.longitude
        self.wp_msg.alt = self._ap_intent.z
        self._wp_sent = False
        self.set_ready_state(True)
        self.log_info("initializing the %s behavior" %self.nodeName)
        return True


    def run_behavior(self):
        ''' Executes one iteration of the behavior
        Sends the waypoint to the autopilot if it has not already been sent.
        Otherwise, the behavior does not need to do anything.
        '''
        if not self._wp_sent:
            self.publishWaypoint(self.wp_msg)
            self._wp_sent = True
            self.log_info("transit to latitude %7.4f, longitude %8.4f, alt %5.1f initiated"\
                          %(self.wp_msg.lat, self.wp_msg.lon, self.wp_msg.alt))


#--------------------------------------------
# Runs a node with the altitude sort behavior
#--------------------------------------------

if __name__ == '__main__':
    rospy.init_node("independent_transit")
    transiter = IndependentTransit("independent_transit")
    transiter.runAsNode(1.0)

