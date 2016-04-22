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


class LossyExchange(object):
    ''' Abstract class used in support of reliable information exchange
    among swarm UAVs relying on lossy communications links

    Member variables:
      _own_id: ID of this UAV
      _subswarm: set of current subswarm IDs
      _crashed: set of UAVs suspected of crashing (no reports)
      _msg_publisher: ROS publisher for swarm behavior messages
      _data: dictionary containing data to be sorted
      _to_send: set of id-value pairs to broadcast
      _lock: re-entrant lock for enforcing thread safety
      _behavior_msg: BehaviorData message object for sending data & requests
      _info_bcast_time: ROS time of the last information broadcast
      _rqst_bcast_time: ROS time of the last information request broadcast      
      rounds: number of rounds the algorithm has executed (for data collection)
      xmit_msgs: number of messages transmitted (for data collection)
      xmit_bytes: number of bytes transmitted (for data collection)

    Class methods:
      reset: initialize to start over
      process_message: process a swarm data message from another UAV
      send_requested: broadcast requested data from this UAV to the network
      decide_sort: broadcast a request for any missing data or perform the sort
    '''

    MIN_COMMS_DELAY = rospy.Duration(0.25) # Throttle comms to limit congestion

    def __init__(self, subswarm_keys, crashed_list, msg_publisher, lock, own_id=1):
        ''' Initializer for the class sets up class variables used for sorting
        @param subswarm_keys: Set object containing active subswarm IDs
        @param crashed_list: Set object containing IDs of possible crashed UAVs
        @param msg_publisher: ROS publisher to the swarm_data_msg topic
        @param lock: reentrant lock that will be used to enforce thread safety
        '''
        if rospy.has_param("aircraft_id"):
            self._own_id = int(rospy.get_param("aircraft_id"))
        else:
            self._own_id = own_id
        self._subswarm = subswarm_keys
        self._crashed = crashed_list
        self._msg_publisher = msg_publisher
        self._lock = lock
        self._data = dict()
        self._to_send = set()
        self._behavior_msg = apmsg.BehaviorParameters()
        self._info_bcast_time = rospy.Time(0.0)
        self._rqst_bcast_time = rospy.Time(0.0)
        self.rounds = 0
        self.xmit_msgs = 0
        self.xmit_bytes = 0


    def reset(self, own_value):
        ''' Initializes the exchange object to start from scratch
        @param own_value: the sort value for this UAV (must be immutable)
        '''
        self._data.clear()
        self._data[self._own_id] = (self._own_id, int(own_value))
        self._to_send.clear()
        self._to_send.add( self._data[self._own_id] )
        self.rounds = 0
        self.xmit_msgs = 0
        self.xmit_bytes = 0


    def process_message(self, msg):
        ''' Processes BehaviorParameter messages from the recv_swarm_data topic
        This method must be implemented by the inheriting class and should be
        called from the ROS message callback.  Received messages can contain
        ID/value pairs or a list of IDs that a particular UAV is requesting the
        sort values for.  Other message types will be ignored.
        '''
        pass


    def send_requested(self):
        ''' Sends any available data that has been requested by other UAVs
        This method must be implemented by the inheriting class and should be
        called once per loop even after decided so that UAVs that have not
        decided yet will still get the info
        '''
        pass


class EagerConsensusSort(LossyExchange):
    ''' Implements a consensus algorithm (more or less) for numerical sorting
    The algorithm sorts UAVs by an arbitrary numerical value that each UAV
    computes on its own behalf.  Sorting is from low to high.
    NOTE: the sorter is set up to sort unsigned shorts to facilitate
          inter-UAV messaging.  All values other than the one for this
          UAV are parsed from BehaviorParameter messages using the
          IdShortValuePairParser, so users of this class only need to
          ensure that the own_value parameter to the reset method is correct

    Member variables:
      _short_list_parser: parser for received id-short value messages
      _needed_parser: parser for generating list of IDs with missing data
      _value_pair_parser: parser object for id_value pairs

    Inherited from LossyExchange:
      _own_id: ID of this UAV
      _subswarm: set of current subswarm IDs
      _crashed: set of UAVs suspected of crashing (no reports)
      _msg_publisher: ROS publisher for swarm behavior messages
      _data: dictionary containing data to be sorted
      _to_send: set of id-value pairs to broadcast
      _lock: re-entrant lock for enforcing thread safety
      _behavior_msg: BehaviorData message object for sending data & requests
      _info_bcast_time: ROS time of the last information broadcast
      _rqst_bcast_time: ROS time of the last information request broadcast      
      rounds: number of rounds the algorithm has executed (for data collection)
      xmit_msgs: number of messages transmitted (for data collection)
      xmit_bytes: number of bytes transmitted (for data collection)

    Class methods:
      process_message: process swarm data message from another UAV (LossyExchange)
      send_requested: broadcast requested data to the network (LossyExchange)
      decide_sort: broadcast a request for any missing data or perform the sort
    '''

    def __init__(self, subswarm_keys, crashed_list, msg_publisher, lock, own_id=1):
        ''' Initializer for the class sets up class variables used for sorting
        @param subswarm_keys: Set object containing active subswarm IDs
        @param crashed_list: Set object containing IDs of possible crashed UAVs
        @param msg_publisher: ROS publisher to the swarm_data_msg topic
        @param lock: reentrant lock that will be used to enforce thread safety
        '''
        LossyExchange.__init__(self, subswarm_keys, crashed_list, msg_publisher, lock, own_id)
        self._short_list_parser = bytes.UShortListParser()
        self._short_list_parser.source_id = self._own_id
        self._value_pair_parser = bytes.IdShortValuePairParser()
        self._value_pair_parser.source_id = self._own_id


    def process_message(self, msg):
        ''' Processes BehaviorParameter messages from the recv_swarm_data topic
        This message should be called from the ROS message callback.
        Received messages can contain ID/value pairs or a list of IDs that a
        particular UAV is requesting the sort values for.  Other message
        types will be ignored.
        '''
        # UAV sent data pairs--if this UAV doesn't have them, add them
        if msg.id == bytes.ID_VALUE_PAIRS:
            with self._lock:
                self._value_pair_parser.unpack(msg.params)
                for pair in self._value_pair_parser.pairs:
                    self._data[pair[0]] = pair

        # UAV needing data for specific UAVs--if this UAV has them, send them
        elif msg.id == bytes.USHORT_LIST:
            with self._lock:
                self._short_list_parser.unpack(msg.params)
                for uav in self._short_list_parser.number_list:
                    if uav in self._data:
                        self._to_send.add(self._data[uav])


    def send_requested(self):
        ''' Sends any available data that has been requested by other UAVs
        This method should be called once per loop even after decided
        so that UAVs that have not decided yet will still get the info
        '''
        t = rospy.Time.now()
        if (t - self._info_bcast_time) < LossyExchange.MIN_COMMS_DELAY:
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
                if uav not in self._data and \
                   uav not in self._crashed:
                    missing_ids.append(uav)

            if len(missing_ids) > 0:
                t = rospy.Time.now()
                if (t - self._rqst_bcast_time) >= \
                   LossyExchange.MIN_COMMS_DELAY:
                    self.rounds += 1
                    self._short_list_parser.number_list = missing_ids
                    self._behavior_msg.id = bytes.USHORT_LIST
                    self._behavior_msg.params = self._short_list_parser.pack()
                    self._msg_publisher.publish(self._behavior_msg)
                    self.xmit_msgs += 1
                    self.xmit_bytes += (len(self._behavior_msg.params) + 4)
                    self._rqst_bcast_time = t
                return None

            lo_to_hi = []
            for uav in self._data:
                if uav in self._subswarm and uav not in self._crashed:
                    lo_to_hi.append(self._data[uav])
            lo_to_hi = sorted(lo_to_hi, key = lambda tup: tup[1])
            return lo_to_hi


class LazyConsensusSort(EagerConsensusSort):
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
            with self._lock:
                self._value_pair_parser.unpack(msg.params)
                for pair in self._value_pair_parser.pairs:
                    self._data[pair[0]] = pair

        # UAV needing data for specific UAVs--if this UAV has them, send them
        elif msg.id == bytes.USHORT_LIST:
            with self._lock:
                self._short_list_parser.unpack(msg.params)
                if self._own_id in self._short_list_parser.number_list:
                    self._to_send.add(self._data[self._own_id])

