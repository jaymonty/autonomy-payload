#!/usr/bin/env python

# Standard Python imports
from argparse import ArgumentParser
import os
import sys
import time

# General ROS imports
import rospy
import std_msgs

# AP-specific imports
import ap_lib.acs_messages as messages
from ap_lib.acs_socket import Socket
from ap_msgs import msg as ap_msg
from ap_srvs import srv as ap_srv
from autopilot_bridge import msg as pilot_msg
from autopilot_bridge import srv as pilot_srv

#-----------------------------------------------------------------------
# Interface between ROS-land and Network-land

class NetworkBridge(object):

    ### Internal utility classes, methods, and data ###

    ROS_BASENAME = 'network'

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

    class _TypeAndObj(object):
        def __init__(self):
            self.type = None
            self.obj = None

    # NOTE: Because this doesn't get used until a publication or service
    # call is actually made, the ROS graph may look more disconnected
    # than it really is at first.
    def _lookupOrAdd(self, o_name, o_type, cons, store):
        if o_name in store:
            if store[o_name].type is not o_type:
                raise Exception("type mismatch for " + o_name)
        else:
            tao = NetworkBridge._TypeAndObj()
            tao.type = o_type
            tao.obj = cons("%s/%s" % (self.ros_basename, o_name), o_type)
            store[o_name] = tao
        return store[o_name].obj

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

    def publish(self, topic, msg):
        try:
            pub = self._lookupOrAdd(topic, type(msg),
                                    rospy.Publisher, self.publishers)
            pub.publish(msg)
        except Exception as ex:
            raise Exception("publish: " + ex.args[0])

    def callService(self, s_name, s_type, **s_fields):
        try:
            # NOTE: If services start failing, consider creating new proxy
            # during each call, or initializing with persistent=True option.
            srv = self._lookupOrAdd(s_name, s_type,
                                    rospy.ServiceProxy, self.service_proxies)
            return srv(**s_fields)
        except Exception as ex:
            raise Exception("callService: " + ex.args[0])

    def setParam(self, name, val):
        # NOTE: Param names are relative to the base namespace (e.g., /)
        try:
            rospy.set_param(name, value)
        except Exception as ex:
            raise Exception("setParam: " + ex.args[0])

    def sendMessage(self, message):
        try:
            self.sock.send(message)
        except Exception as ex:
            raise Exception("sendMessage: " + ex.args[0])

    def setSubswarmID(self, subswarm_id):
        self.sock.subswarm = subswarm_id
        rospy.set_param('subswarm_id', subswarm_id)

    ### Add handlers for ROS, network, and timed events ###

    def addNetHandler(self, m_type, m_func, log_success=True):
        def wrapper(msg):
            m_name = m_type.__name__
            try:
                m_func(msg, self)
                if log_success:
                    rospy.loginfo("NET %s" % m_name)
            except Exception as ex:
                rospy.logwarn("NET ERROR %s: %s" % (m_name, ex.args[0]))
        self.msg_handlers[m_type] = wrapper

    def addSubHandler(self, m_topic, m_type, m_func, log_success=False):
        def wrapper(msg):
            try:
                m_func(msg, self)
                if log_success:
                    rospy.loginfo("SUB %s" % m_topic)
            except Exception as ex:
                rospy.logwarn("SUB ERROR %s: %s" % (m_topic, ex.args[0]))
        rospy.Subscriber("%s/%s" % (self.ros_basename, m_topic), m_type, wrapper)

    def addTimedHandler(self, hz, callback):
        def wrapper():
            try:
                callback(bridge)
            except Exception as ex:
                rospy.logwarn("TIMED ERROR: " + ex.args[0])
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

        # Initialize subswarm ID
        try:
            subswarm_id = 0
            if rospy.has_param('subswarm_id'):
                subswarm_id = rospy.get_param('subswarm_id')
            self.setSubswarmID(subswarm_id)
        except Exception as ex:
            rospy.logwarn("Could not set subswarm_id: " + ex.args[0])

        # Initialize stores for handlers and ROS objects
        self.msg_handlers = {}
        self.publishers = {}
        self.service_proxies = {}
        self.timed_events = []

    def runLoop(self, loop_rate):
        rate = rospy.Rate(loop_rate)
        while not rospy.is_shutdown():
            try: # NOTE: This outer try is extra safety, shouldn't need it
                # Handle time-driven events
                t = time.time()
                for ev in [ev for ev in self.timed_events if ev.due(t)]:
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
                rospy.logwarn("runLoop error: " + ex.args[0])

#-----------------------------------------------------------------------
# Timed event handlers

def timed_status(bridge):
    message = messages.FlightStatus()
    message.msg_dst = Socket.ID_BCAST_ALL  # TODO: Send to ground

    # Populate with default values
    message.msg_secs = 0
    message.msg_nsecs = 0
    message.mode = 15  # TODO: Use enum (this is the unknown value)
    message.armed = False
    message.ok_ahrs = False
    message.ok_as = False
    message.ok_gps = False
    message.ok_ins = False
    message.ok_mag = False
    message.ok_pwr = False
    message.gps_sats = 0
    message.batt_rem = 0
    message.batt_vcc = 0
    message.batt_cur = 0
    message.airspeed = 0
    message.alt_rel = 0
    message.gps_hdop = 0
    message.mis_cur = 0
    message.ctl_mode = 0
    message.ctl_ready = [0] * 16  # TODO: Don't hardcode

    # If we have valid data, populate correctly
    if timed_status.c_status:
        message.ctl_mode = timed_status.c_status.state.active_controller
        for c in timed_status.c_status.state.controllers:
            if c.controller_id <= len(message.ctl_ready):
                message.ctl_ready[c.controller_id] = c.is_ready
    if timed_status.f_status:
        message.msg_secs = timed_status.f_status.header.stamp.secs
        message.msg_nsecs = timed_status.f_status.header.stamp.nsecs
        message.mode = timed_status.f_status.mode
        message.armed = timed_status.f_status.armed
        message.ok_ahrs = timed_status.f_status.ahrs_ok
        message.ok_as = timed_status.f_status.as_ok
        message.ok_gps = timed_status.f_status.gps_ok
        message.ok_ins = timed_status.f_status.ins_ok
        message.ok_mag = timed_status.f_status.mag_ok
        message.ok_pwr = timed_status.f_status.pwr_ok
        message.gps_sats = timed_status.f_status.gps_sats
        message.batt_rem = timed_status.f_status.pwr_batt_rem
        message.batt_vcc = timed_status.f_status.pwr_batt_vcc
        message.batt_cur = timed_status.f_status.pwr_batt_cur
        message.airspeed = timed_status.f_status.as_read
        message.alt_rel = timed_status.f_status.alt_rel
        message.gps_hdop = timed_status.f_status.gps_eph
        message.mis_cur = timed_status.f_status.mis_cur

    # Populate flight-ready flag from param
    message.ready = rospy.has_param("flight_ready") and \
                    bool(rospy.get_param("flight_ready"))

    # Add friendly name
    message.name = bridge.ac_name

    # Send it!
    bridge.sendMessage(message)

timed_status.c_status = None  # Controller status
timed_status.f_status = None  # Flight status

#-----------------------------------------------------------------------
# ROS subscription handlers

def sub_subswarm_id(msg, bridge):
    bridge.setSubswarmID(msg.data)

def sub_controller_status(msg, bridge):
    # Just update; timed event will do the send
    timed_status.c_status = msg

def sub_flight_status(msg, bridge):
    # Just update; timed event will do the send
    timed_status.f_status = msg

def sub_pose(msg, bridge):
    message = messages.Pose()
    message.msg_dst = Socket.ID_BCAST_ALL
    message.msg_secs = msg.header.stamp.secs
    message.msg_nsecs = msg.header.stamp.nsecs
    message.lat = msg.pose.pose.position.lat
    message.lon = msg.pose.pose.position.lon
    message.alt = msg.pose.pose.position.alt  # NOTE: send MSL
    message.q_x = msg.pose.pose.orientation.x
    message.q_y = msg.pose.pose.orientation.y
    message.q_z = msg.pose.pose.orientation.z
    message.q_w = msg.pose.pose.orientation.w
    message.vlx = msg.twist.twist.linear.x
    message.vly = msg.twist.twist.linear.y
    message.vlz = msg.twist.twist.linear.z
    message.vax = msg.twist.twist.angular.x
    message.vay = msg.twist.twist.angular.y
    message.vaz = msg.twist.twist.angular.z
    bridge.sendMessage(message)

#-----------------------------------------------------------------------
# Network receive handlers

def net_pose(message, bridge):
    msg = ap_msg.SwarmVehicleState()
    msg.vehicle_id = message.msg_src
    msg.subswarm_id = message.msg_sub
    msg.state.header.stamp.secs = message.msg_secs
    msg.state.header.stamp.nsecs = message.msg_nsecs
    msg.state.header.seq = 0
    msg.state.pose.pose.position.lat = message.lat
    msg.state.pose.pose.position.lon = message.lon
    msg.state.pose.pose.position.alt = message.alt
    msg.state.pose.pose.position.rel_alt = 0
    msg.state.pose.pose.position.using_alt = True
    msg.state.pose.pose.position.using_rel_alt = False
    msg.state.pose.pose.orientation.x = message.q_x
    msg.state.pose.pose.orientation.y = message.q_y
    msg.state.pose.pose.orientation.z = message.q_z
    msg.state.pose.pose.orientation.w = message.q_w
    # msg.state.pose.covariance is not used
    msg.state.twist.twist.linear.x = message.vlx
    msg.state.twist.twist.linear.y = message.vly
    msg.state.twist.twist.linear.z = message.vlz
    msg.state.twist.twist.angular.x = message.vax
    msg.state.twist.twist.angular.y = message.vay
    msg.state.twist.twist.angular.z = message.vaz
    # msg.state.twist.covariance is not used
    bridge.publish('recv_pose', msg)

def net_heartbeat(message, bridge):
    msg = pilot_msg.Heartbeat()
    msg.counter = message.counter
    bridge.publish('recv_heart_ground', msg)

def net_arm(message, bridge):
    msg = std_msgs.msg.Bool()
    msg.data = message.enable
    bridge.publish('recv_arm', msg)

def net_mode(message, bridge):
    msg = std_msgs.msg.UInt8()
    msg.data = message.mode
    bridge.publish('recv_mode', msg)

def net_land(message, bridge):
    msg = std_msgs.msg.Empty()
    bridge.publish('recv_land', msg)

def net_land_abort(message, bridge):
    msg = std_msgs.msg.UInt16()
    msg.data = message.alt
    bridge.publish('recv_land_abort', msg)

def net_guided_goto(message, bridge):
    msg = pilot_msg.LLA()
    msg.lat = message.lat
    msg.lon = message.lon
    msg.alt = message.alt
    bridge.publish('recv_guided_goto', msg)

def net_waypoint_goto(message, bridge):
    msg = std_msgs.msg.UInt16()
    msg.data = message.index
    bridge.publish('recv_waypoint_goto', msg)

def net_slave_setup(message, bridge):
    bridge.callService('slave_setup', pilot_srv.SlaveSetup,
                       enable=message.enable, channel=message.channel)

def net_flight_ready(message, bridge):
    bridge.setParam('flight_ready', message.ready)

def net_subswarm_id(message, bridge):
    bridge.setSubswarmID(message.subswarm)
    msg = std_msgs.msg.UInt8()
    msg.data = sub_id
    bridge.publish('update_subswarm', msg)

def net_controller_mode(message, bridge):
    bridge.callService('controller_mode', ap_srv.SetInteger,
                       setting=message.controller)

def net_follower_set(message, bridge):
    msg = ap_msg.FormationOrderStamped()
    msg.header.seq = message.seq
    msg.header.stamp = rospy.Time(message.msg_secs, message.msg_nsecs)
    msg.header.frame_id = 'base_footprint'
    msg.order.leader_id = message.leader_id
    msg.order.range = message.follow_range
    msg.order.angle = message.offset_angle
    msg.order.alt_mode = message.alt_mode
    msg.order.control_alt = message.control_alt
    bridge.publish('recv_follower_set', msg)

def net_sequencer_set(message, bridge):
    msg = ap_msg.WaypointListStamped()
    msg.header.seq = message.seq
    msg.header.stamp = rospy.Time(message.msg_secs, message.msg_nsecs)
    msg.header.frame_id = 'base_footprint'
    for wp in message.wp_list:
        lla = pilot_msg.LLA()
        lla.lat = wp[0]
        lla.lon = wp[1]
        lla.alt = wp[2]
        msg.waypoints.append(lla)
    bridge.publish('recv_sequencer_set', msg)

def net_health_state(message, bridge):
    bridge.callService('health_state', ap_srv.SetBoolean,
                       enable=message.enable)

def net_shutdown(message, bridge):
    res = os.system("sudo halt")
    if res != 0:
        raise Exception

#-----------------------------------------------------------------------
# Main section

if __name__ == '__main__':
    # Grok args
    parser = ArgumentParser(sys.argv[0] + ' [options]')
    parser.add_argument("--id", dest="acid", type=int, default=None,
                        help="Aircraft ID (default to ROS param aircraft_id)")
    parser.add_argument("--name", dest="acname", default=None,
                        help="Aircraft Name (default to ROS param aircraft_name)")
    parser.add_argument("--device", dest="device", default=None,
                        help="Network device to use (default wlan0)")
    parser.add_argument("--port", dest="port", type=int, default=None,
                        help="UDP port (default 5554)")
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    try:
        # Initialize the bridge
        bridge = NetworkBridge(args.acid, args.acname, args.port, args.device)

        # Set up handlers
        bridge.addTimedHandler(2.0, timed_status)
        bridge.addSubHandler('update_subswarm',
                             std_msgs.msg.UInt8, sub_subswarm_id)
        bridge.addSubHandler('update_ctlr_status',
                             ap_msg.ControllerGroupStateStamped,
                             sub_controller_status)
        bridge.addSubHandler('update_flight_status',
                             pilot_msg.Status, sub_flight_status)
        bridge.addSubHandler('send_pose', pilot_msg.Geodometry, sub_pose)
        bridge.addNetHandler(messages.Pose, net_pose,
                             log_success=False)
        bridge.addNetHandler(messages.Heartbeat, net_heartbeat,
                             log_success=False)
        bridge.addNetHandler(messages.Arm, net_arm)
        bridge.addNetHandler(messages.Mode, net_mode)
        bridge.addNetHandler(messages.Land, net_land)
        bridge.addNetHandler(messages.LandAbort, net_land_abort)
        bridge.addNetHandler(messages.GuidedGoto, net_guided_goto)
        bridge.addNetHandler(messages.WaypointGoto, net_waypoint_goto)
        bridge.addNetHandler(messages.SlaveSetup, net_slave_setup)
        bridge.addNetHandler(messages.FlightReady, net_flight_ready)
        bridge.addNetHandler(messages.SetSubswarm, net_subswarm_id)
        bridge.addNetHandler(messages.SetController, net_controller_mode)
        bridge.addNetHandler(messages.FollowerSetup, net_follower_set)
        bridge.addNetHandler(messages.WPSequencerSetup, net_sequencer_set)
        bridge.addNetHandler(messages.PayloadHeartbeat, net_health_state)
        bridge.addNetHandler(messages.PayloadShutdown, net_shutdown)

        # Run the loop (shouldn't stop until node is shut down)
        print "\nStarting network bridge loop...\n"
        bridge.runLoop(100)
    except Exception as ex:
        print "NETWORK BRIDGE FATAL ERROR: " + ex.args[0]
        # If rospy happened to get initialized before the error, log there too
        rospy.logfatal(ex.args[0])
