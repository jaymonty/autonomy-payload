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


class AltitudeSort(Behavior):
    ''' Simple behavior that generates a sorted altitude order using
    the ConsensusSorter object.  This behavior does not actually
    provide any control, and simply collects data.  The UAV should
    just orbit at the standby point while the consensus algorithm
    computes the altitude order.

    Class member variables:
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
        Behavior.__init__(self, nodename, enums.ALTITUDE_SORT)
        self._own_uav_id = rospy.get_param("aircraft_id")
        self._sorter = dist.ConsensusSort(self._subswarm_keys, \
                                          self._crashed_keys, \
                                          self._behaviorDataPublisher, \
                                          self._swarm_lock)
        self._done = False
        self._algorithm = enums.ALTITUDE_SORT
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
        self.subscribe_to_swarm()
        self._sorter.reset(self._ap_intent.z)
        self._done = False
        self.set_ready_state(True)
        self.log_info("initializing the %s behavior" %self.nodeName)
        return True


    def run_behavior(self):
        ''' Executes one iteration of the behavior
        First state is order determination.  After the order has been determined
        intercept waypoints will be generated for each iteration
        '''
        self._sorter.send_requested()
        if not self._done:
            order = self._sorter.decide_sort()
            if order != None:
                self._done = True
                self.log_info("sort complete in %d rounds: %s" \
                              %(self._sorter.rounds, str(order)))


    def _process_swarm_data_msg(self, dataMsg):
        ''' Processes swarm data messages received over the network
        @param dataMsg: message containing received behavior data
        '''
        if self.is_active:
            self._sorter.process_message(dataMsg)


    def set_active(self, activate):
        ''' Override of the Behavior method
        Not usually a good idea to override this method, but this just collects
        and logs data upon deactivation and calls the parent class method
        @param activate: Boolean value to activate or deactivate the behavior
        @return active status upon method completion
        '''
        if not activate and self.is_active:
            self.log_info("%s summary: Done=%s, Rounds=%d, Messages=%d, Bytes=%d" \
                          %(self.nodeName, str(self._done), self._sorter.rounds, \
                            self._sorter.xmit_msgs, self._sorter.xmit_bytes))
            parser = bytes.ConsensusSummaryParser()
            parser.algorithm = self._algorithm
            parser.rounds = self._sorter.rounds
            parser.messages = self._sorter.xmit_msgs
            parser.bytes = self._sorter.xmit_bytes
            msg = apmsg.BehaviorParameters()
            msg.id = bytes.CONSENSUS_SUMMARY
            msg.params = parser.pack()
            self._behaviorDataPublisher.publish(msg)
        return super(AltitudeSort, self).set_active(activate)


#--------------------------------------------
# Runs a node with the altitude sort behavior
#--------------------------------------------

if __name__ == '__main__':
    rospy.init_node("altitude_sort")
    sorter = AltitudeSort("altitude_sort")
    sorter.runAsNode(2.0)

