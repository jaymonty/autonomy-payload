#!/usr/bin/env python

# Standard Python imports
from argparse import ArgumentParser
import os
import sys

# General ROS imports
import rospy
import std_msgs
from std_srvs import srv as std_srv

# AP-specific imports
import ap_lib.acs_messages as messages
from ap_lib.acs_socket import Socket
from ap_lib.acs_network import NetworkBridge
from ap_msgs import msg as ap_msg
from ap_srvs import srv as ap_srv
from autopilot_bridge import msg as pilot_msg
from autopilot_bridge import srv as pilot_srv

#-----------------------------------------------------------------------
# Timed event handlers
# NOTE: Be sure to add a handler in the "main" code below

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
    message.swarm_state = timed_status.swarm_state
    message.swarm_behavior = timed_status.swarm_behavior
    message.batt_rem = 0
    message.batt_vcc = 0
    message.batt_cur = 0
    message.airspeed = 0
    message.alt_rel = 0
    message.gps_hdop = 0
    message.mis_cur = 0
    message.ctl_mode = 0
    message.ctl_ready = [0] * 17  # TODO: Don't hardcode

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
        message.ok_ahrs = timed_status.f_status.ahrs_ok and \
                          bool(timed_status.calgyros_done)
        message.ok_as = timed_status.f_status.as_ok and \
                        bool(timed_status.calpress_done)
        message.ok_gps = timed_status.f_status.gps_ok and \
                         (timed_status.f_status.gps_sats >= 6)  # Current reqt
        message.ok_ins = timed_status.f_status.ins_ok
        message.ok_mag = timed_status.f_status.mag_ok
        message.ok_pwr = timed_status.f_status.pwr_ok
        message.batt_rem = timed_status.f_status.pwr_batt_rem
        message.batt_vcc = timed_status.f_status.pwr_batt_vcc
        message.batt_cur = timed_status.f_status.pwr_batt_cur
        message.airspeed = timed_status.f_status.as_read
        message.alt_rel = timed_status.f_status.alt_rel
        message.gps_hdop = timed_status.f_status.gps_eph
        message.mis_cur = timed_status.f_status.mis_cur

    # Populate param-based fields
    def getboolparam(p):
        if rospy.has_param(p):
            return bool(rospy.get_param(p))
        return False
    message.ready = getboolparam("flight_ready")
    message.ok_prm = getboolparam("ok_param")
    message.ok_fen = getboolparam("ok_fence")
    message.ok_ral = getboolparam("ok_rally")
    message.ok_wp = getboolparam("ok_wp")

    # Add friendly name
    message.name = bridge.ac_name

    # Send it!
    bridge.sendMessage(message)

timed_status.c_status = None        # Controller status
timed_status.f_status = None        # Flight status
timed_status.swarm_state = 0        # Current swarm state
timed_status.swarm_behavior = 0     # Currently active swarm behavior
timed_status.calpress_done = False  # Airspeed calibration done?
timed_status.calgyros_done = True   # Gyros calibration done?

#-----------------------------------------------------------------------
# ROS subscription handlers
# NOTE: Be sure to add a handler in the "main" code below

def sub_subswarm_id(msg, bridge):
    bridge.setSubswarmID(msg.data)

def sub_controller_status(msg, bridge):
    # Just update; timed event will do the send
    timed_status.c_status = msg

def sub_flight_status(msg, bridge):
    # Just update; timed event will do the send
    timed_status.f_status = msg

def sub_swarm_state(msg, bridge):
    # Just update the swarm_state; timed event will do the send
    timed_status.swarm_state = msg.data

def sub_swarm_behavior(msg, bridge):
    # Just update the swarm_behavior; timed event will do the send
    timed_status.swarm_behavior = msg.data

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
# NOTE: Be sure to add a handler in the "main" code below
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
    # NOTE: adjust queue_size as necessary
    bridge.publish('recv_pose', msg, queue_size=50)

def net_auto_status(message, bridge):
    msg = ap_msg.SwarmControlState()
    msg.vehicle_id = message.msg_src
    msg.subswarm_id = message.msg_sub
    msg.swarm_state = message.swarm_state
    msg.swarm_behavior = message.swarm_behavior
    msg.active_controller = message.ctl_mode
    msg.autopilot_mode = message.mode
    bridge.publish('recv_swarm_ctl_state', msg, queue_size = 50)

def net_heartbeat(message, bridge):
    msg = pilot_msg.Heartbeat()
    msg.counter = message.counter
    bridge.publish('recv_heart_ground', msg)

def net_arm(message, bridge):
    msg = std_msgs.msg.Bool()
    msg.data = message.enable
    bridge.publish('recv_arm', msg, latched=True)

def net_mode(message, bridge):
    msg = std_msgs.msg.UInt8()
    msg.data = message.mode
    bridge.publish('recv_mode', msg, latched=True)

def net_land(message, bridge):
    msg = std_msgs.msg.Empty()
    bridge.publish('recv_land', msg, latched=True)

def net_land_abort(message, bridge):
    msg = std_msgs.msg.UInt16()
    msg.data = message.alt
    bridge.publish('recv_land_abort', msg, latched=True)

def net_guided_goto(message, bridge):
    msg = pilot_msg.LLA()
    msg.lat = message.lat
    msg.lon = message.lon
    msg.alt = message.alt
    bridge.publish('recv_guided_goto', msg, latched=True)

def net_waypoint_goto(message, bridge):
    msg = std_msgs.msg.UInt16()
    msg.data = message.index
    bridge.publish('recv_waypoint_goto', msg, latched=True)

def net_slave_setup(message, bridge):
    bridge.callService('slave_setup', pilot_srv.SlaveSetup,
                       enable=message.enable, channel=message.channel)

def net_flight_ready(message, bridge):
    bridge.setParam('flight_ready', message.ready)

def net_subswarm_id(message, bridge):
    msg = std_msgs.msg.UInt8()
    msg.data = message.subswarm
    bridge.publish('recv_subswarm', msg, latched=True)

def net_swarm_behavior(message, bridge):
    bridge.callService('set_swarm_behavior', ap_srv.SetInteger,
                       setting=message.swarm_behavior)

def net_swarm_state(message, bridge):
    bridge.callService('set_swarm_state', ap_srv.SetInteger,
                       setting=message.swarm_state)

# Possible candidate for deprication
def net_controller_mode(message, bridge):
    bridge.callService('controller_mode', ap_srv.SetInteger,
                       setting=message.controller)

# Possible candidate for deprication
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
    bridge.publish('recv_follower_set', msg, latched=True)

# Possible candidate for deprication
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
    bridge.publish('recv_sequencer_set', msg, latched=True)

def net_calibrate(message, bridge):
    def main():
        try:
            if message.index == 1:
                srvname = 'cal_pressure'
                timeout = 5.0
                timed_status.calpress_done = False
            elif message.index == 2:
                srvname = 'cal_gyros'
                timeout = 10.0
                timed_status.calgyros_done = False
            else:
                raise Exception("invalid calibration type")
            res = bridge.callService(srvname, pilot_srv.TimedAction,
                                     timeout=timeout)
            # If pressure, update flag for status message
            if message.index == 1:
                timed_status.calpress_done = bool(res.ok)
            elif message.index == 2:
                timed_status.calgyros_done = bool(res.ok)
        except Exception as ex:
            raise Exception("net_calibrate: " + str(ex.args[0]))
        finally:
            net_calibrate.active = False
    def error():
        net_calibrate.active = False
    if net_calibrate.active:
        raise Exception("calibration currently in progress")
    net_calibrate.active = True
    bridge.doInThread(main, error)
net_calibrate.active = False

def net_demo(message, bridge):
    def main():
        if message.demo == 1:
            srvname = 'demo_servos'
        elif message.demo == 2:
            srvname = 'demo_motor'
        else:
            raise Exception("invalid demo type")
        bridge.callService(srvname, std_srv.Empty)
        net_demo.active = False
    def error():
        net_demo.active = False
    if net_demo.active:
        raise Exception("demo currently in progress")
    net_demo.active = True
    bridge.doInThread(main, error)
net_demo.active = False

def net_mission_config(message, bridge):
    def main():
        # Reset OK flags so users know work is in progress
        for cfg in ['rally', 'wp', 'param']:
            rospy.set_param('ok_' + cfg, False)

        # Replace names with values
        # NOTE: This is a direct string replacement of KEY with VALUE.
        replacements = {}
        replacements['STDALT'] = message.std_alt

        # Select from among options
        # NOTE: This looks for lines starting with 'KEY_', keeping those that
        # start with 'KEY_VALUE ' (but eliminating that string) and deleting
        # other matching lines.
        selections = {}
        selections['STACK'] = message.stack_num

        # Set parameters
        plist = []
        plist.append(pilot_msg.ParamPair('ALT_HOLD_RTL',
                                         replacements['STDALT'] * 100.0))
        plist.append(pilot_msg.ParamPair('FENCE_RETALT',
                                         replacements['STDALT']))
        res = bridge.callService('param_setlist',
                                 pilot_srv.ParamSetList,
                                 param=plist)
        rospy.set_param('ok_param', res.ok)

        # Build out templates and load them
        for cfg in ['rally', 'wp']:
            # Set up locations
            base_file = os.path.expanduser("~/blessed/%s.template" % cfg)
            temp_file = base_file + ".tmp"

            # Modify blessed file into temp location
            with open(base_file, 'r') as inf, open(temp_file, 'w') as outf:
                for line in inf:
                    if not line or not isinstance(line, str):
                        continue

                    # Line selection
                    # NOTE: Can only have one selection pattern per line
                    select = ''
                    for pattern, value in selections.iteritems():
                        if not line.startswith(pattern):
                            continue
                        select = "%s_%s " % (pattern, str(value))
                        break
                    if select:
                        # If wrong selection, omit the line
                        if not line.startswith(select):
                            continue
                        # If right selection, strip the string and use it
                        line = line.replace(select, '')

                    # String replacement
                    # NOTE: May be multiple replacements per line
                    for pattern, value in replacements.iteritems():
                        line = line.replace(str(pattern), str(value))

                    outf.write(line)

            # Call load service
            res = bridge.callService('load_' + cfg,
                                     pilot_srv.FileLoad,
                                     name=temp_file)

            # Update OK flag
            rospy.set_param('ok_' + cfg, res.ok)

            # Clean up temp file
            os.remove(temp_file)

        # Reset active flag
        net_mission_config.active = False

    def error():
        net_mission_config.active = False

    if net_mission_config.active:
        raise Exception("configuration currently in progress")
    net_mission_config.active = True
    bridge.doInThread(main, error)
net_mission_config.active = False

# Highest-numbered are administrative/debug messages

def net_ap_reboot(message, bridge):
    msg = std_msgs.msg.Empty()
    bridge.publish('recv_ap_reboot', msg, latched=True)

def net_health_state(message, bridge):
    bridge.callService('health_state', ap_srv.SetBoolean,
                       enable=message.enable)

def net_shutdown(message, bridge):
    res = os.system("sudo halt")
    if res != 0:
        raise Exception("could not halt")

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

        # NOTE: If we need to stand up any ROS publishers in advance,
        # make calls to bridge.addPublisher() here.

        # Set up handlers
        # NOTE: the add*Handler() methods handle ROS object creation;
        # see their definitions above for details and assumptions.
        bridge.addTimedHandler(2.0, timed_status)
        bridge.addSubHandler('update_subswarm',
                             std_msgs.msg.UInt8, sub_subswarm_id)
        bridge.addSubHandler('swarm_behavior',
                             std_msgs.msg.UInt8, sub_swarm_behavior)
        bridge.addSubHandler('update_ctlr_status',
                             ap_msg.ControllerGroupStateStamped,
                             sub_controller_status)
        bridge.addSubHandler('update_flight_status',
                             pilot_msg.Status, sub_flight_status)
        bridge.addSubHandler('send_pose', pilot_msg.Geodometry, sub_pose)
        bridge.addSubHandler('swarm_state', std_msgs.msg.UInt8, sub_swarm_state)
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
        bridge.addNetHandler(messages.SwarmBehavior, net_swarm_behavior)
        bridge.addNetHandler(messages.SwarmState, net_swarm_state)
        bridge.addNetHandler(messages.SetController, net_controller_mode) #depricate?
        bridge.addNetHandler(messages.FollowerSetup, net_follower_set) #depricate?
        bridge.addNetHandler(messages.WPSequencerSetup, net_sequencer_set) #depricate?
        bridge.addNetHandler(messages.Calibrate, net_calibrate)
        bridge.addNetHandler(messages.Demo, net_demo)
        bridge.addNetHandler(messages.MissionConfig, net_mission_config)
        bridge.addNetHandler(messages.AutopilotReboot, net_ap_reboot)
        bridge.addNetHandler(messages.PayloadHeartbeat, net_health_state)
        bridge.addNetHandler(messages.PayloadShutdown, net_shutdown)
        bridge.addNetHandler(messages.FlightStatus, net_auto_status)

        # Run the loop (shouldn't stop until node is shut down)
        print "\nStarting network bridge loop...\n"
        bridge.runLoop(50)
    except Exception as ex:
        print "NETWORK BRIDGE FATAL ERROR: " + str(ex.args[0])
        # If rospy happened to get initialized before the error, log there too
        rospy.logfatal(str(ex.args[0]))

