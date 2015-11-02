#!/usr/bin/env python

# Standard Python imports
from argparse import ArgumentParser
import os
import sys, time

# General ROS imports
import rospy
import std_msgs
from std_srvs import srv as std_srv

# AP-specific imports
import ap_lib.acs_messages as messages
from ap_lib.acs_socket import Socket
from ap_lib.acs_network import NetworkBridge
from ap_lib import ap_enumerations as enums
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
    message.fence_state = 2
    message.batt_rem = 0
    message.batt_vcc = 0
    message.batt_cur = 0
    message.airspeed = 0
    message.alt_rel = 0
    message.gps_hdop = 0
    message.mis_cur = 0
    message.ctl_ready = [0] * 17  # TODO: Don't hardcode

    # If we have valid data, populate correctly
    if timed_status.c_status:
        for c in timed_status.c_status.state.behaviors:
            if c.behavior_id <= len(message.ctl_ready):
                message.ctl_ready[c.behavior_id] = c.is_ready
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
        message.fence_state = timed_status.f_status.fence_status
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

def sub_behavior_summary(msg, bridge):
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

def sub_swarm_behavior_data(msg, bridge):
    message = messages.SwarmBehaviorData()
    t = rospy.Time.now()
    message.msg_dst = Socket.ID_BCAST_ALL
    message.msg_secs = t.secs
    message.msg_nsecs = t.nsecs
    message.data_type = msg.id
    message.data = msg.params
    bridge.sendMessage(message)

def sub_intent(msg, bridge):
    message = messages.VehicleIntent()
    message.msg_dst = Socket.ID_BCAST_ALL
    message.swarm_behavior = msg.swarm_behavior
    message.lat = msg.loc.lat
    message.lon = msg.loc.lon
    message.alt = msg.loc.alt
    bridge.sendMessage(message)

#-----------------------------------------------------------------------
# Network receive handlers
# NOTE: Be sure to add a handler in the "main" code below
def net_pose(message, bridge):
    msg = ap_msg.SwarmVehicleState()
    msg.vehicle_id = message.msg_src
    msg.subswarm_id = message.msg_sub
    msg.received_at = rospy.Time.now()  # TODO: get from socket lib
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

def net_weather_update(message, bridge):
    msg = pilot_msg.WeatherData()
    msg.baro_millibars = message.baro
    msg.temp_C = message.temperature
    msg.wind_mph = message.wind_speed
    msg.wind_direction = message.wind_direction
    bridge.publish('recv_weather', msg, latched=True)

def net_req_prev_n_ap_msgs(message, bridge):
    def req_response_thread():
        try:
            ret = bridge.callService('ap_msg_queue_last_n',
               pilot_srv.ReqPrevNMsgs, n=message.n,since_seq=message.since_seq)
        except Exception as e:
            net_req_prev_n_ap_msgs.active = False
            raise e
            return

        final_seq = 0
        if ret.msgs != []:
            final_seq = ret.msgs[0].seq

        response = messages.PrevMsgAP()
        response.msg_dst = Socket.ID_BCAST_ALL
        # Populate with default values
        response.msg_secs = 0
        response.msg_nsecs = 0

        response.final_seq = final_seq
    
        for m in ret.msgs:
            response.seq = m.seq
            response.msg = m.text

            try:
                bridge.sendMessage(response)
            except Exception as e:
                net_req_prev_n_ap_msgs.active = False
                raise e
                return

            #rate-limit this traffic
            time.sleep(0.05)

        net_req_prev_n_ap_msgs.active = False
    def error():
        net_req_prev_n_ap_msgs.active = False

    #No need to throw exception if active, we actually expect multiple
    #GCSs to attempt to request msgs from the same plane in the near future.
    #Just assuming each GCS is after the same messages if the requests occur
    #simultaneously.  The broadcast responses will get to all GCSs.
    if net_req_prev_n_ap_msgs.active is False:
        net_req_prev_n_ap_msgs.active = True
        bridge.doInThread(req_response_thread, error)
net_req_prev_n_ap_msgs.active=False

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
    bridge.callService('set_subswarm', ap_srv.SetInteger,
                       setting=message.subswarm)

def net_swarm_behavior(message, bridge):

    # Process differently based on ordered type (behavior, pause, suspend)
    if type(message) == messages.SuspendSwarmBehavior:
        behavior_msg = ap_msg.BehaviorParameters()
        behavior_msg.id = enums.SWARM_STANDBY
        behavior_msg.params = []
        bridge.callService('run_behavior', ap_srv.SetBehavior,\
                           params=behavior_msg)

    # This one will go away once all of the behavior-specific
    # messages are fully incorporated into SwarmCommander
    elif type(message) == messages.SwarmBehavior:
        behavior_msg = ap_msg.BehaviorParameters()
        behavior_msg.id = message.swarm_behavior
        behavior_msg.params = message.swarm_parameters
        bridge.callService('run_behavior', ap_srv.SetBehavior,\
                           params=behavior_msg)

    elif type(message) == messages.PauseSwarmBehavior:
        bridge.callService('pause_behavior', ap_srv.SetBoolean,
                           enable=message.behavior_pause)

def net_swarm_behavior_data(message, bridge):
    behavior_msg = ap_msg.BehaviorParameters()
    behavior_msg.id = message.data_type
    behavior_msg.params = message.data
    bridge.publish('recv_swarm_data', behavior_msg, latched=True)

def net_swarm_state(message, bridge):
    bridge.callService('set_swarm_state', ap_srv.SetInteger,
                       setting=message.swarm_state)

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
            raise Exception("net_calibrate: " + str(ex))
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
        # NOTE: Save param flag so we don't "ok" it if verify failed
        old_ok_param = rospy.has_param('ok_param') and \
                       rospy.get_param('ok_param')

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
        # NOTE: Only "ok" if prior verify worked AND these sets worked
        rospy.set_param('ok_param', res.ok and old_ok_param)

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

        # Make sure waypoint 1 is reloaded on autopilot
        msg = std_msgs.msg.UInt16()
        msg.data = 1
        bridge.publish('recv_waypoint_goto', msg, latched=True)

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
        bridge.addSubHandler('subswarm_id',
                             std_msgs.msg.UInt8, sub_subswarm_id)
        bridge.addSubHandler('swarm_behavior',
                             std_msgs.msg.UInt8, sub_swarm_behavior)
        bridge.addSubHandler('behavior_summary',
                             ap_msg.BehaviorGroupStateStamped,
                             sub_behavior_summary)
        bridge.addSubHandler('update_flight_status',
                             pilot_msg.Status, sub_flight_status,
                             log_success=False)
        bridge.addSubHandler('send_pose', pilot_msg.Geodometry, sub_pose)
        bridge.addSubHandler('swarm_state', std_msgs.msg.UInt8, sub_swarm_state)
        bridge.addSubHandler('send_swarm_behavior_data', \
                             ap_msg.BehaviorParameters, \
                             sub_swarm_behavior_data)
        bridge.addSubHandler('payload_intent', ap_msg.VehicleIntent, sub_intent)
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
        bridge.addNetHandler(messages.SuspendSwarmBehavior, net_swarm_behavior)
        bridge.addNetHandler(messages.PauseSwarmBehavior, net_swarm_behavior)
        bridge.addNetHandler(messages.SwarmBehaviorData, net_swarm_behavior_data)
        bridge.addNetHandler(messages.SwarmState, net_swarm_state)
        bridge.addNetHandler(messages.Calibrate, net_calibrate)
        bridge.addNetHandler(messages.Demo, net_demo)
        bridge.addNetHandler(messages.MissionConfig, net_mission_config)
        bridge.addNetHandler(messages.AutopilotReboot, net_ap_reboot)
        bridge.addNetHandler(messages.PayloadHeartbeat, net_health_state)
        bridge.addNetHandler(messages.PayloadShutdown, net_shutdown)
        bridge.addNetHandler(messages.FlightStatus, net_auto_status,
                             log_success=False)
        bridge.addNetHandler(messages.WeatherData, net_weather_update)
        bridge.addNetHandler(messages.ReqPrevNMsgsAP, net_req_prev_n_ap_msgs)

        # Run the loop (shouldn't stop until node is shut down)
        print "\nStarting network bridge loop...\n"
        bridge.runLoop(50)
    except Exception as ex:
        print "NETWORK BRIDGE FATAL ERROR: " + str(ex)
        # If rospy happened to get initialized before the error, log there too
        rospy.logfatal(str(ex))
