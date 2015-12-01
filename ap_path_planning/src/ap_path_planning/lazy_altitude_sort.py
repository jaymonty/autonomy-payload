#!/usr/bin/env python

# Simple behavior for consensus sort algorithm data collection
#
# Duane Davis 2015

# ROS imports
import rospy
import std_msgs.msg as stdmsg

# ACS imports
import ap_msgs.msg as apmsg
import ap_lib.ap_enumerations as enums
import ap_lib.distributed_algorithms as dist
import ap_lib.bitmapped_bytes as bytes
from ap_lib.behavior import *
import ap_path_planning.altitude_sort as alt_sort


class LazyAltitudeSort(alt_sort.AltitudeSort):
    ''' Simple behavior that generates a sorted altitude order using
    the LazyConsensusSorter object.  This behavior does not actually
    provide any control, and simply collects data.  The UAV should
    just orbit at the standby point while the consensus algorithm
    computes the altitude order.

    Inherited from AltitudeSorter:
      _own_uav_id: ID of this aircraft
      _sorter: object implementing the consensus sort algorithm
      _done: set to True when the sort is complete

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
        Behavior.__init__(self, nodename, enums.LAZY_ALTITUDE_SORT)
        self._own_uav_id = rospy.get_param("aircraft_id")
        self._sorter = dist.LazyConsensusSort(self._subswarm_keys, \
                                              self._crashed_keys, \
                                              self._behaviorDataPublisher, \
                                              self._swarm_lock)
        self._done = False
        self._algorithm = enums.LAZY_ALTITUDE_SORT
#        self.DBUG_PRINT = True
#        self.INFO_PRINT = True
#        self.WARN_PRINT = True


#--------------------------------------------
# Runs a node with the altitude sort behavior
#--------------------------------------------

if __name__ == '__main__':
    rospy.init_node("lazy_altitude_sort")
    sorter = LazyAltitudeSort("lazy_altitude_sort")
    sorter.runAsNode(2.0)

