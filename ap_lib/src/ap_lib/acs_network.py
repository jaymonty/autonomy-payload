#!/usr/bin/env python

import rospy
import threading
import time

from ap_lib.acs_socket import Socket

#-----------------------------------------------------------------------
# Interface between ROS-land and Network-land

class NetworkBridge(object):

    ### Internal utility classes, methods, and data ###

    ROS_BASENAME = 'network'

    # Container for timed event callbacks, with timing logic
    class _TimedEvent(object):
        def __init__(self, interval, callback):
            self._interval = interval
            self._callback = callback
            self._next_time = time.time() + self._interval

        def due(self, t=None):
            if not t:
                t = time.time()
            return bool(self._next_time <= t)

        def run(self, t=None):
            self._callback()
            if not t:
                t = time.time()
            self._next_time = t + self._interval

    # Basic two-object container for lookups
    class _TypeAndObj(object):
        def __init__(self, t=None, o=None):
            self.type = t
            self.obj = o

    # Prefer provided args, then ROS params, then default values, then FAIL
    def _getArg(self, arg, ros_param, default, error_text='<unknown>'):
        if arg:
            return arg
        elif rospy.has_param(ros_param):
            return rospy.get_param(ros_param)
        elif default:
            return default
        else:
            raise Exception("Could not find setting for " + error_text)

    ### Utility functions for handlers ###

    # NOTE: This is here so we can pre-create publishers if needed
    def addPublisher(self, topic, m_type, queue_size=1, latched=False):
        try:
            # See if a publisher of the correct type exists, or make one
            if topic not in self.publishers:
                pub = rospy.Publisher(self.ros_basename + '/' + topic,
                                      m_type,
                                      tcp_nodelay=True,
                                      latch=latched,
                                      queue_size=queue_size)
                tao = NetworkBridge._TypeAndObj(m_type, pub)
                self.publishers[topic] = tao
            elif self.publishers[topic].type != m_type:
                raise Exception("type mismatch for " + topic)
        except Exception as ex:
            raise Exception("addPublisher: " + str(ex.args[0]))

    def publish(self, topic, msg, queue_size=1, latched=False):
        try:
            # Make sure the publisher exists
            pub = self.addPublisher(topic, type(msg), queue_size, latched)

            # Publish the message
            self.publishers[topic].obj.publish(msg)
        except Exception as ex:
            raise Exception("publish: " + str(ex.args[0]))

    # NOTE: Optional error_cb fires if any of the tries below fail (use care!)
    def doInThread(self, main_cb, error_cb=None):
        def wrapper():
            try:
                main_cb()
            except Exception as ex:
                if callable(error_cb): error_cb()
                rospy.logwarn("THREAD ERROR: " + str(ex.args[0]))
        try:
            # NOTE: t might get gc'ed before the thread finishes
            t = threading.Thread(target=wrapper)
            t.setDaemon(True)  # Allows termination with outstanding threads
            t.start()
        except Exception as ex:
            if callable(error_cb): error_cb()
            raise Exception("doInThread: " + str(ex.args[0]))

    def callService(self, s_name, s_type, **s_fields):
        try:
            # NOTE: It appears safer to re-lookup the service each time,
            # though if the service provider is super-stable it may be
            # worth looking at persistent service connections.
            srv = rospy.ServiceProxy(self.ros_basename + '/' + s_name,
                                     s_type)
            return srv(**s_fields)
        except Exception as ex:
            raise Exception("callService: " + str(ex.args[0]))

    def setParam(self, name, value):
        # NOTE: Param names are relative to the base namespace (e.g., /)
        try:
            rospy.set_param(name, value)
        except Exception as ex:
            raise Exception("setParam: " + str(ex.args[0]))

    def sendMessage(self, message):
        try:
            self.sock.send(message)
        except Exception as ex:
            raise Exception("sendMessage: " + str(ex.args[0]))

    def setSubswarmID(self, subswarm_id):
        # Set subswarm ID in socket so it knows which subswarm-destined
        # broadcasts to accept
        self.sock.subswarm = subswarm_id

    ### Add handlers for ROS, network, and timed events ###

    def addNetHandler(self, m_type, m_func, log_success=True):
        def wrapper(msg):
            m_name = m_type.__name__
            try:
                m_func(msg, self)
                if log_success:
                    rospy.loginfo("NET %s" % m_name)
            except Exception as ex:
                rospy.logwarn("NET ERROR %s: %s" % (m_name, str(ex.args[0])))
        self.msg_handlers[m_type] = wrapper

    def addSubHandler(self, m_topic, m_type, m_func, log_success=False):
        def wrapper(msg):
            try:
                m_func(msg, self)
                if log_success:
                    rospy.loginfo("SUB %s" % m_topic)
            except Exception as ex:
                rospy.logwarn("SUB ERROR %s: %s" % (m_topic, str(ex.args[0])))
        rospy.Subscriber("%s/%s" % (self.ros_basename, m_topic), m_type, wrapper)

    def addTimedHandler(self, hz, callback):
        def wrapper():
            try:
                callback(self)
            except Exception as ex:
                rospy.logwarn("TIMED ERROR: " + str(ex.args[0]))
        interval = 1.0 / float(hz)  # NOTE: 0 Hz is illegal anyway
        self.timed_events.append(NetworkBridge._TimedEvent(interval, wrapper))

    ### Startup and Main loop ###

    def __init__(self, ac_id=None, ac_name=None,
                 net_port=None, net_dev=None,
                 ros_basename=None):

        # Initialize ROS first
        if not ros_basename:
            ros_basename = NetworkBridge.ROS_BASENAME
        rospy.init_node(ros_basename)
        self.ros_basename = rospy.get_name()

        # Get socket options and set up ACS Socket
        try:
            self.ac_id = self._getArg(ac_id, 'aircraft_id', None, 'Aircraft ID')
            self.ac_name = self._getArg(ac_name, 'aircraft_name', None, 'Aircraft Name')
            self.net_dev = self._getArg(net_dev, 'network_device', 'wlan0', None)
            self.net_port = self._getArg(net_port, 'network_port', 5554, None)

            # NOTE: This is a bit of a hack
            local_ip, remote_ip = None, None
            if self.net_dev == 'lo':
                local_ip, remote_ip = '127.0.0.1', '127.0.1.1'

            print "\nStarting Network <-> ROS interface with following settings:" \
                + "\n  device:\t\t" + str(self.net_dev) \
                + "\n  port:\t\t\t" + str(self.net_port)

            self.sock = Socket(self.ac_id, self.net_port, self.net_dev,
                               local_ip, remote_ip)
        except Exception as ex:
            raise ex

        # Initialize subswarm ID - socket library can filter messages destined for
        # subswarms; need to keep socket synced with aircraft's subswarm ID
        try:
            subswarm_id = 0

            # TODO: If param goes away, this can too
            if rospy.has_param('subswarm_id'):
                subswarm_id = rospy.get_param('subswarm_id')

            self.setSubswarmID(subswarm_id)
        except Exception as ex:
            rospy.logwarn("Could not set subswarm_id: " + str(ex.args[0]))

        # Initialize stores for handlers and ROS objects
        self.msg_handlers = {}
        self.publishers = {}
        self.timed_events = []

    def runLoop(self, loop_rate):
        rate = rospy.Rate(loop_rate)
        while not rospy.is_shutdown():
            try: # NOTE: This outer try is extra safety, shouldn't need it
                # Handle time-driven events
                t = time.time()
                for ev in self.timed_events:
                    if ev.due(t):
                        # NOTE: technically, want to use the time when the event
                        # actually ran, but this is slightly more efficient.
                        ev.run(t)

                # Receive a message
                msg = self.sock.recv()

                if msg == False:  # Got a message, but invalid or not for us
                    continue      # May be more messages in the queue
                if msg == None:   # No messages available
                    rate.sleep()  # Wait a little bit, then continue
                    continue

                # Process the message
                #print "Got Message: " + str(type(msg))
                if type(msg) in self.msg_handlers:
                    self.msg_handlers[type(msg)](msg)

            except Exception as ex:
                try:
                    print "runLoop error"  # try to get some info out
                    rospy.logwarn("runLoop error: " + str(ex.args[0]))
                except:
                    pass  # Don't log if it causes another error

