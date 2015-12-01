#!/usr/bin/env python

#-----------------------------------------------------------------------
# distributed_algorithms.py
# Duane Davis, 2015
#
# Defines classes and functions for the implementation of distributed
# algorithms for use in ACS modules as required
#-----------------------------------------------------------------------

# Standard Python imports
import threading

# ROS imports
import rospy

# ACS imports
import ap_msgs.msg as apmsg
import ap_lib.bitmapped_bytes as bytes


class ConsensusSort(object):
    ''' Implements a consensus algorithm (more or less) for numerical sorting
    The algorithm sorts UAVs by an arbitrary numerical value that each UAV
    computes on its own behalf.  Sorting is from low to high.
    NOTE: the sorter is set up to sort unsigned shorts to facilitate
          inter-UAV messaging.  All values other than the one for this
          UAV are parsed from BehaviorParameter messages using the
          IdShortValuePairParser, so users of this class only need to
          ensure that the own_value parameter to the reset method is correct

    Member variables:
      _own_id: ID of this UAV
      _subswarm: set of current subswarm IDs
      _crashed: set of UAVs suspected of crashing (no reports)
      _msg_publisher: ROS publisher for swarm behavior messages
      _sort_data: dictionary containing data to be sorted
      _to_send: set of id-value pairs to broadcast
      _short_list_parser: parser for received id-short value messages
      _needed_parser: parser for generating list of IDs with missing data
      _value_pair_parser: parser object for id_value pairs
      _info_bcast_time: ROS time of the last information broadcast
      _rqst_bcast_time: ROS time of the last information request broadcast      
      _lock: re-entrant lock for enforcing thread safety
      rounds: number of rounds the algorithm has executed (for data collection)
      xmit_msgs: number of messages transmitted (for data collection)
      xmit_bytes: number of bytes transmitted (for data collection)

    Class methods:
      reset: initialize to start a search from nothing
      process_message: process a swarm data message from another UAV
      send_requested: broadcast requested data from this UAV to the network
      decide_sort: broadcast a request for any missing data or perform the sort
    '''

    MIN_COMMS_DELAY = rospy.Duration(0.25) # Throttle comms to limit congestion

    def __init__(self, subswarm_keys, crashed_list, msg_publisher, lock):
        ''' Initializer for the class sets up class variables used for sorting
        @param subswarm_keys: Set object containing active subswarm IDs
        @param crashed_list: Set object containing IDs of possible crashed UAVs
        @param msg_publisher: ROS publisher to the swarm_data_msg topic
        @param lock: reentrant lock that will be used to enforce thread safety
        '''
        self._own_id = int(rospy.get_param("aircraft_id"))
        self._subswarm = subswarm_keys
        self._crashed = crashed_list
        self._msg_publisher = msg_publisher
        self._sort_data = dict()
        self._to_send = set()
        self._short_list_parser = bytes.UShortListParser()
        self._short_list_parser.source_id = self._own_id
        self._value_pair_parser = bytes.IdShortValuePairParser()
        self._value_pair_parser.source_id = self._own_id
        self._behavior_msg = apmsg.BehaviorParameters()
        self._lock = lock
        self._info_bcast_time = rospy.Time(0.0)
        self._rqst_bcast_time = rospy.Time(0.0)
        self.rounds = 0
        self.xmit_msgs = 0
        self.xmit_bytes = 0


    def reset(self, own_value):
        ''' Initializes the sorter to start a new sort from scratch
        @param own_value: the sort value for this UAV
        '''
        self._sort_data.clear()
        self._sort_data[self._own_id] = (self._own_id, int(own_value))
        self._to_send.clear()
        self._to_send.add( self._sort_data[self._own_id] )


    def process_message(self, msg):
        ''' Processes BehaviorParameter messages from the recv_swarm_data topic
        This message should be called from the ROS message callback.
        Received messages can contain ID/value pairs or a list of IDs that a
        particular UAV is requesting the sort values for.  Other message
        types will be ignored.
        '''
        # UAV sent data pairs--if this UAV doesn't have them, add them
        if msg.id == bytes.ID_VALUE_PAIRS:
            self._value_pair_parser.unpack(msg.params)
            with self._lock:
                for pair in self._value_pair_parser.pairs:
                    self._sort_data[pair[0]] = pair

        # UAV needing data for specific UAVs--if this UAV has them, send them
        elif msg.id == bytes.USHORT_LIST:
            self._short_list_parser.unpack(msg.params)
            with self._lock:
                for uav in self._short_list_parser.number_list:
                    if uav in self._sort_data:
                        self._to_send.add(self._sort_data[uav])


    def send_requested(self):
        ''' Sends any available data that has been requested by other UAVs
        This method should be called once per loop even after decided
        so that UAVs that have not decided yet will still get the info
        '''
        t = rospy.Time.now()
        if (t - self._info_bcast_time) < ConsensusSort.MIN_COMMS_DELAY:
            return
        with self._lock:
            self._value_pair_parser.pairs = list(self._to_send)
            if len(self._value_pair_parser.pairs) > 0:
                self._behavior_msg.id = bytes.ID_VALUE_PAIRS
                self._behavior_msg.params = self._value_pair_parser.pack()
                self._msg_publisher.publish(self._behavior_msg)
                self._to_send.clear()
                self.xmit_msgs += 1
                self.xmit_bytes += (len(self._behavior_msg.params) + 4)
                self._info_bcast_time = t


    def decide_sort(self):
        ''' Sorts the list if all data is available or requests missing data
        @return the sorted id-value list or None if more data is still needed
        '''
        missing_ids = []
        with self._lock:
            for uav in self._subswarm:
                if uav not in self._sort_data and \
                   uav not in self._crashed:
                    missing_ids.append(uav)

            if len(missing_ids) > 0:
                t = rospy.Time.now()
                if (t - self._rqst_bcast_time) >= ConsensusSort.MIN_COMMS_DELAY:
                    self.rounds += 1
                    self._short_list_parser.number_list = missing_ids
                    self._behavior_msg.id = bytes.USHORT_LIST
                    self._behavior_msg.params = self._short_list_parser.pack()
                    self._msg_publisher.publish(self._behavior_msg)
                    self._rqst_bcast_time = t
                return None

            lo_to_hi = []
            for uav in self._sort_data:
                if uav in self._subswarm and uav not in self._crashed:
                    lo_to_hi.append(self._sort_data[uav])
            lo_to_hi = sorted(lo_to_hi, key = lambda tup: tup[1])
            return lo_to_hi


class LazyConsensusSort(ConsensusSort):
    ''' Implements a consensus algorithm (more or less) for numerical sorting
    The algorithm differs from the above consensus sort algorithm in that it
    only broadcasts its own information when another UAV needs it.
    '''

    def process_message(self, msg):
        ''' Processes BehaviorParameter messages from the recv_swarm_data topic
        This message should be called from the ROS message callback.
        Received messages can contain ID/value pairs or a list of IDs that a
        particular UAV is requesting the sort values for.  Other message
        types will be ignored.
        '''
        # UAV sent data pairs--if this UAV doesn't have them, add them
        if msg.id == bytes.ID_VALUE_PAIRS:
            self._value_pair_parser.unpack(msg.params)
            with self._lock:
                for pair in self._value_pair_parser.pairs:
                    self._sort_data[pair[0]] = pair

        # UAV needing data for specific UAVs--if this UAV has them, send them
        elif msg.id == bytes.USHORT_LIST:
            self._short_list_parser.unpack(msg.params)
            with self._lock:
                if self._own_id in self._short_list_parser.number_list:
                    self._to_send.add(self._sort_data[self._own_id])

