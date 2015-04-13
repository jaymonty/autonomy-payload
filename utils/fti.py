#!/usr/bin/env python

# Super-hackish Flight Tech Interface and Preflight System
# Mike Clement, 2014-2015

# TODO: This is super ugly, no separation of concerns, yada yada
# Redesign so we have a legitimate model-view-controller

from PySide.QtCore import *
from PySide.QtGui import *

import ap_lib.acs_messages as messages
from ap_lib.acs_socket import Socket

from math import asin, atan2, degrees
from optparse import OptionParser
import subprocess
import sys
import time

''' Classes '''

# Simple representation of UAV state
class UAVState():
    def __init__(self, text, color):
        self.text = text
        self.color = color

# A QListWidgetItem for UAVs, with some custom data and printing
class UAVListWidgetItem(QListWidgetItem):
    # Standard item attributes
    FONT = QFont('Helvetica', 14)

    # UAV state
    STATE_NONE = UAVState('INVALID', QBrush(QColor('white')))
    STATE_OFFLINE = UAVState('OFFLINE', QBrush(QColor('red')))
    STATE_WAITING_AP = UAVState('WAITING AP', QBrush(QColor('cyan').darker(150)))
    STATE_PREFLIGHT = UAVState('PREFLIGHT', QBrush(QColor('yellow')))
    STATE_READY= UAVState('READY', QBrush(QColor.fromRgb(0,200,0)))
    STATE_FLYING = UAVState('FLYING', QBrush(QColor.fromRgb(160,160,160)))

    def __init__(self, *args, **kwargs):
        QListWidgetItem.__init__(self, *args, **kwargs)

        # Attributes of an item
        self._msg_s = None   # Last-received FlightStatus message
        self._msg_p = None   # Last-received Pose message
        self._time = 0.0     # Time last message was received
        self._state = None   # State (color coding)
        self._aspd = []      # History of airspeed readings

        self.setFont(self.FONT)

    def __str__(self):
        if self._state is None or self._msg_s is None:
            return "UNKNOWN"
        return "%s : %s (%d / %s / %2.3fv)" % \
               (self._msg_s.name,
                self._state.text,
                self._msg_s.msg_src,
                self._msg_s.msg_src_ip,
                self._msg_s.batt_vcc / 1000.0)

    # Make explicit the ordering for sorted lists
    def __lt__(self, other):
        if not self.getID(): return True
        if not other.getID(): return False
        return bool(self.getID() < other.getID())

    def getID(self):
        if self._msg_s:
            return self._msg_s.msg_src
        return None

    def getIP(self):
        if self._msg_s:
            return self._msg_s.msg_src_ip
        return None

    def getTime(self):
        return self._time

    def getState(self):
        return self._state

    def getLastStatus(self):
        return self._msg_s

    def getLastPose(self):
        return self._msg_p

    def getAvgAspd(self):
        if self._msg_s is None: return None
        if len(self._aspd) < 10: return None
        return float(sum(self._aspd)) / float(len(self._aspd))

    def getMaxAspd(self):
        if self._msg_s is None: return None
        return float(max(self._aspd))

    # Set the state (color)
    def setState(self, state):
        self._state = state
        if self._state:
            self.setBackground(self._state.color)
        else:
            self.setBackground(self.STATE_NONE.color)

    def processPose(self, msg):
        self._msg_p = msg
        # TODO: Decide what "offline" means if we see poses but not status

    # Process a status message
    def processStatus(self, msg):
        # Update internal data
        self._msg_s = msg
        self._time = time.time()

        # Update airspeed history
        while len(self._aspd) > 10: self._aspd.pop(0)
        self._aspd.append(msg.airspeed)

        # Determine color coding / "state"
        if msg.armed and msg.alt_rel > 20000:
            # Armed ^ (Alt > 20m AGL) -> Active/Flying
            state = self.STATE_FLYING
        elif msg.batt_vcc == 0.0 and msg.mode == 15:
            # No voltage ^ Unknown mode -> No autopilot data yet
            state = self.STATE_WAITING_AP
        elif msg.ready:
            state = self.STATE_READY
        else:
            state = self.STATE_PREFLIGHT
        self.setState(state)

        # Update UI
        status_text = "%s (%d / %2.3fv)" % \
                      (msg.name,
                       msg.msg_src,
                       msg.batt_vcc / 1000.0)
        if msg.armed:
            status_text += " <ARM>"
        if msg.mode == 4:
            status_text += " <AUTO>"
        self.setText(status_text)

# This creates a list box populated by listening to ACS messages
class UAVListWidget(QListWidget):

    # Number of times to resend messages (in lieu of reliability)
    SEND_RETRY = 3

    # Time (in seconds) we start culling aircraft
    OFFLINE_TIME = 5.0
    DELETE_TIME = 30.0

    def __init__(self, sock, filter_states=[], parent=None):
        QListWidget.__init__(self, parent)

        # Set up a timer for handling network data
        self.timer = QTimer(self)
        self.connect(self.timer, SIGNAL("timeout()"), self.handleTimer)
        self.timer.start(100)

        # Filter out aircraft in certain states
        self.filter_states = filter_states

        # Add a dummy aircraft to "select none"
        self.dummy = UAVListWidgetItem()
        self.dummy.setText(" <<None>>")
        self.addItem(self.dummy)

        # Add state for MAVProxy connections
        self.mav_popen = None
        self.mav_id = None
        self.mav_channel = None
        self.mav_start_time = None

        # ACS protocol connection
        self._sock = sock

        # Lights
        self.lights = {}
        self.lights_sta = False
        self.lights_cfg = False
        self.lights_imu = False

    ''' API Extensions '''

    # Return the *singly-selected* list item
    def currentItem(self):
        sel = self.selectedItems()
        # Second half of condition is false if None or 0 (dummy)
        if len(sel) != 1 or not sel[0].getID():
            return None
        return sel[0]

    # Return item matching a given ident
    def itemByIdent(self, ident):
        for i in range(self.count()):
            if self.item(i).getID() == ident:
                return self.item(i)
        return None

    ''' Convenience methods '''

    # Check that selected item is valid and in acceptable state
    def _checkItemState(self, ok_states=[]):
        item = self.currentItem()
        if item is None or not item.getID():
            print "No entry selected"
            return None
        if ok_states != [] and item.getState() not in ok_states:
            print "Can't do that to a %s aircraft!" % item.getState().text
            return None
        return item

    # Pop up a message box
    def _okBox(self, text):
        mbx = QMessageBox()
        mbx.setText(text)
        mbx.setStandardButtons(QMessageBox.Ok)
        mbx.setDefaultButton(QMessageBox.Ok)
        mbx.exec_()

    # Pop up a confirmation box and return boolean result
    def _confirmBox(self, text):
        mbx = QMessageBox()
        mbx.setText(text)
        mbx.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        mbx.setDefaultButton(QMessageBox.No)
        return (mbx.exec_() == QMessageBox.Yes)

    # Repeatedly send a message (in lieu of reliability)
    def _sendMessage(self, msg):
        for i in range(self.SEND_RETRY):
            self._sock.send(msg)
            time.sleep(0.01)

    # Tear down a mavproxy session
    def _destroySlaveChannel(self):
        if self.mav_channel is None or self.mav_id is None:
            return

        ss = messages.SlaveSetup()
        ss.msg_dst = int(self.mav_id)
        ss.msg_secs = 0
        ss.msg_nsecs = 0
        ss.enable = False
        ss.channel = self.mav_channel

        # Send the message a few times (in lieu of reliability)
        self._sendMessage(ss)

        self.mav_id = None
        self.mav_channel = None

    ''' Status and IMU lights '''

    class _QLight(QLabel):

        def __init__(self, name):
            QLabel.__init__(self, name)

        def unknown(self):
            self.setStyleSheet("QLabel { color : black; }")

        def bad(self):
            self.setStyleSheet("QLabel { color : red; font: bold; }")

        def ok(self):
            self.setStyleSheet("QLabel { color : orange; font: bold; }")

        def good(self):
            self.setStyleSheet("QLabel { color : green; }")

        def setbool(self, b_good, b_ok=False):
            if b_good: self.good()
            elif b_ok: self.ok()
            else: self.bad()

    def _buildLights(self, light_list):
        layout = QHBoxLayout()
        for l in light_list:
            lbl = self._QLight(l)
            self.lights[l] = lbl
            layout.addWidget(lbl)
        return layout

    LIGHT_STA_OK = ['ahrs', 'as', 'gps', 'ins', 'mag', 'pwr']
    LIGHT_STA_CP = ['aspd', 'batt', 'ralt']
    LIGHT_STA = sorted(LIGHT_STA_OK + LIGHT_STA_CP)
    LIGHT_CFG_OK = ['ral', 'wp', 'fen', 'prm']
    LIGHT_CFG_CP = ['fw', 'sw', 'repo']
    LIGHT_CFG = LIGHT_CFG_CP + LIGHT_CFG_OK
    LIGHT_IMU = ['UP', 'DOWN', 'LEFT', 'RIGHT']

    def buildStatusLights(self):
        self.lights_sta = True
        return self._buildLights(self.LIGHT_STA)

    def buildConfigLights(self):
        self.lights_cfg = True
        return self._buildLights(self.LIGHT_CFG)

    def buildImuLights(self):
        self.lights_imu = True
        return self._buildLights(self.LIGHT_IMU)

    def updateStatusLights(self):
        # If no aircraft is selected or aircraft is online,
        # blank out ALL lights
        item = self.currentItem()
        if item is None or \
           item.getLastStatus() is None or \
           item.getState() == UAVListWidgetItem.STATE_OFFLINE:
            for light in self.lights:
                self.lights[light].unknown()
            return

        # Done if not using lights based on status message
        if not self.lights_sta and not self.lights_cfg:
            return

        # Update all lights based on OK flags
        msg = item.getLastStatus()
        for light in self.LIGHT_STA_OK + self.LIGHT_CFG_OK:
            if light not in self.lights:
                continue
            self.lights[light].setbool(getattr(msg, 'ok_'+light))

        # Update remaining (computed) lights

        # Relative alt must be +/- 10 m
        self.lights['ralt'].setbool(-10000.0 < msg.alt_rel < 10000.0)

        # Average airspeed (over 10 samples) must have *some* deviation,
        # but not too much
        as_avg = item.getAvgAspd()
        # Max airspeed used so we can check positive pressure on the ground
        as_max = item.getMaxAspd()
        if item.getState() == UAVListWidgetItem.STATE_FLYING:
            # If flying, ignore pressure
            self.lights['aspd'].good()
        elif as_max and as_max > 20.0:
            # Allow for positive pressure check
            self.lights['aspd'].ok()
        else:
            # Allow for zeroize + slight noise check
            self.lights['aspd'].setbool(as_avg and 0.0 < as_avg < 4.0)

        # Battery voltage should be > 12.4 V at takeoff
        # using "ok" color to distinguish from actual problem
        vcc = msg.batt_vcc
        self.lights['batt'].setbool(vcc > 12400, vcc > 11000)

    def updatePoseLights(self):
        THRESH = 30.0  # Minimum angle in degrees

        # If invalid to update this aircraft, just return
        # (updateStatusLights() handles this case for us)
        item = self.currentItem()
        if not self.lights_imu or \
           item is None or \
           item.getLastPose() is None or \
           item.getState() == UAVListWidgetItem.STATE_OFFLINE:
            return

        # Get pose and calculate Euler angles from quaternions
        msg = item.getLastPose()
        # NOTE: Code based on ArduPilot's AP_Math/quaternion.cpp
        (q2, q3, q4, q1) = (msg.q_x, msg.q_y, msg.q_z, msg.q_w)
        roll = degrees(atan2(2.0*(q1*q2 + q3*q4), 1 - 2.0*(q2*q2 + q3*q3)))
        pitch = degrees(asin(2.0*(q1*q3 - q4*q2)))
        yaw = degrees(atan2(2.0*(q1*q4 + q2*q3), 1 - 2.0*(q3*q3 + q4*q4)))

        # Color lights RED if beyond threshold angle
        self.lights['UP'].setbool(pitch < THRESH)
        self.lights['DOWN'].setbool(pitch > -THRESH)
        self.lights['LEFT'].setbool(roll > -THRESH)
        self.lights['RIGHT'].setbool(roll < THRESH)

    ''' Handle timer '''

    # Periodic event to look for new UAVs and to update the list
    def handleTimer(self):

        # If a subprocess is open, see if we can cull it
        if self.mav_popen is not None:
            p = self.mav_popen.poll()
            if p is not None:
                self._destroySlaveChannel()
                del self.mav_popen
                self.mav_popen = None
                print "Ended MAVProxy session; " \
                    + "was open for %0.03f seconds and returned code %d" \
                    % (time.time()-self.mav_start_time, p)

        # Handle received messages
        while True:
            # Break if no message, skip non-FlightStatus messages
            msg = self._sock.recv()
            if msg is None:
                break
            if not (isinstance(msg, messages.FlightStatus) or \
                    isinstance(msg, messages.Pose)):
                continue

            # Look up item or create it
            # NOTE: Only create when we see a status message
            item = self.itemByIdent(msg.msg_src)
            if item is None and isinstance(msg, messages.FlightStatus):
                item = UAVListWidgetItem()
                item.setHidden(True)
                self.addItem(item)
            if item is None:
                continue

            # Let the item process the message
            if isinstance(msg, messages.FlightStatus):
                item.processStatus(msg)
                if item is self.currentItem():
                    self.updateStatusLights()
            elif isinstance(msg, messages.Pose):
                item.processPose(msg)
                if item is self.currentItem():
                    self.updatePoseLights()

        # Cull those we haven't seen in a while
        cur_time = time.time()
        for i in range(self.count()):
            if not self.item(i).getID():
                # Ignore the "none" item
                continue
            if self.item(i).getState() in self.filter_states:
                self.item(i).setHidden(True)
                self.setCurrentItem(self.dummy)
            elif self.item(i).getTime() < (cur_time - self.DELETE_TIME):
                self.item(i).setHidden(True)
                self.setCurrentItem(self.dummy)
            elif self.item(i).getTime() < (cur_time - self.OFFLINE_TIME):
                self.item(i).setState(UAVListWidgetItem.STATE_OFFLINE)
            else:
                self.item(i).setHidden(False)

        # schedule refresh and raise event
        self.update()
        self.itemSelectionChanged.emit()

    ''' Task handlers '''

    def handleMAVProxy(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_PREFLIGHT,
                                     UAVListWidgetItem.STATE_READY,
                                     UAVListWidgetItem.STATE_FLYING])
        if item is None:
            return

        # Can only support one MAVProxy subprocess (for now)
        if self.mav_popen is not None:
            print "Please close your other MAVProxy instance first"
            return

        print "Starting MAVProxy session"

        # Pick an aircraft-unique port
        slave_port = 15554 + item.getID()
        self.mav_id = item.getID()
        self.mav_channel = "udp:%s:%u" % (self._sock._ip, slave_port)

        # Open a slave mavlink channel to the aircraft
        # NOTE: This is done unreliably, so it might fail and we won't know :(
        ss = messages.SlaveSetup()
        ss.msg_dst = int(item.getID())
        ss.msg_secs = 0
        ss.msg_nsecs = 0
        ss.enable = True
        ss.channel = self.mav_channel
        self._sendMessage(ss)

        # Wait a moment so the aircraft can (hopefully) set up the channel
        time.sleep(1)

        # Start up a MAVProxy instance and connect to slave channel
        try:
            self.mav_start_time = time.time()
            self.mav_popen = subprocess.Popen( \
                "xterm -e mavproxy.py --master %s --load-module preflight" % \
                    self.mav_channel,
                shell=True,
                cwd="/tmp")
        except Exception as ex:
            print "Could not start MAVProxy session: " + str(ex.args[0])
            self._destroySlaveChannel()
            self.mav_popen = None

    def handleCalpress(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_PREFLIGHT,
                                     UAVListWidgetItem.STATE_READY])
        if item is None:
            return

        self._okBox("Please cover the pitot tube.")

        # Calibrate the airspeed sensor
        cl = messages.Calibrate()
        cl.msg_dst = int(item.getID())
        cl.msg_secs = 0
        cl.msg_nsecs = 0
        cl.index = 1
        self._sendMessage(cl)

        self._okBox("Please wait for the 'as' and 'aspd' lights to become green.")

    def handleCalgyros(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_PREFLIGHT,
                                     UAVListWidgetItem.STATE_READY])
        if item is None:
            return

        self._okBox("Please keep the aircraft still.")

        # Calibrate the gyros
        cl = messages.Calibrate()
        cl.msg_dst = int(item.getID())
        cl.msg_secs = 0
        cl.msg_nsecs = 0
        cl.index = 2
        self._sendMessage(cl)

        self._okBox("Please wait for the 'ahrs' light to become green.")

    def handleServos(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_PREFLIGHT])
        if item is None:
            return

        # Demo the servos
        dm = messages.Demo()
        dm.msg_dst = int(item.getID())
        dm.msg_secs = 0
        dm.msg_nsecs = 0
        dm.demo = 1  # ID for servo demo
        self._sendMessage(dm)

        # The first time running a demo will take a few seconds.
        # We may want to inform the user so they don't overclick
        # (which in theory shouldn't hurt anything)
        #self._okBox("Please wait ...")

    def handleMotor(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_PREFLIGHT])
        if item is None:
            return
        if not item.getLastStatus().armed:
            self._okBox("Aircraft must be ARMED first")
            return

        if not self._confirmBox("WARNING: Run motor for aircraft %d?" % item.getID()):
            return

        # Demo the motor (must be armed)
        dm = messages.Demo()
        dm.msg_dst = int(item.getID())
        dm.msg_secs = 0
        dm.msg_nsecs = 0
        dm.demo = 2  # ID for motor demo
        self._sendMessage(dm)

    def handleArm(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_PREFLIGHT,
                                     UAVListWidgetItem.STATE_READY])
        if item is None:
            return

        # Arm the motor
        ad = messages.Arm()
        ad.msg_dst = int(item.getID())
        ad.msg_secs = 0
        ad.msg_nsecs = 0
        ad.enable = True         # Boolean
        self._sendMessage(ad)

    def handleDisArm(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_PREFLIGHT,
                                     UAVListWidgetItem.STATE_READY])
        if item is None:
            return

        # Disarm motor
        ad = messages.Arm()
        ad.msg_dst = int(item.getID())
        ad.msg_secs = 0
        ad.msg_nsecs = 0
        ad.enable = False         # Boolean
        self._sendMessage(ad)

    def handleAPReboot(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_PREFLIGHT,
                                     UAVListWidgetItem.STATE_READY])
        if item is None:
            return

        if not self._confirmBox("Reboot autopilot for aircraft %d?" % item.getID()):
            return

        # Reboot the autopilot
        rb = messages.AutopilotReboot()
        rb.msg_dst = int(item.getID())
        rb.msg_secs = 0
        rb.msg_nsecs = 0

        self._sendMessage(rb)

    def handleFlightReady(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_PREFLIGHT,
                                     UAVListWidgetItem.STATE_READY])
        if item is None:
            return

        # Toggle the aircraft to be flight-ready, or not
        fr = messages.FlightReady()
        fr.msg_dst = int(item.getID())
        fr.msg_secs = 0
        fr.msg_nsecs = 0
        fr.ready = (item.getState() == UAVListWidgetItem.STATE_PREFLIGHT)
        self._sendMessage(fr)

    def handleSwarmReady(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_READY,  # For dev
                                     UAVListWidgetItem.STATE_FLYING])
        if item is None:
            return

        if not self._confirmBox("Confirm aircraft %d?" % item.getID()):
            return

        # Also send (FOR FX20) to the staging (ingress) waypoint
        wg = messages.WaypointGoto()
        wg.msg_dst = int(item.getID())
        wg.msg_secs = 0
        wg.msg_nsecs = 0
        wg.index = 3
        self._sendMessage(wg)

    def handleAUTO(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_READY,
                                     UAVListWidgetItem.STATE_FLYING])
        if item is None:
            return

        # Get and set desired WP #
        try:
            wp_des = int(lnWP.text())
        except:
            print "You must supply a valid waypoint!"
            return
        print "WP is %d" % wp_des
        wpa = messages.WaypointGoto()
        wpa.msg_dst = int(item.getID())
        wpa.msg_secs = 0
        wpa.msg_nsecs = 0
        wpa.index = wp_des
        self._sendMessage(wpa)

        # Trigger AUTO
        au = messages.Mode()
        au.msg_dst = int(item.getID())
        au.msg_secs = 0
        au.msg_nsecs = 0
        au.mode = 4           # AUTO mode number

        self._sendMessage(au)

    def handleRTL(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_FLYING])
        if item is None:
            return

        # Trigger RTL
        rtl = messages.Mode()
        rtl.msg_dst = int(item.getID())
        rtl.msg_secs = 0
        rtl.msg_nsecs = 0
        rtl.mode = 0           # RTL mode number

        self._sendMessage(rtl)

    def handleLand(self, approach):
        item = self._checkItemState([UAVListWidgetItem.STATE_FLYING])
        if item is None:
            return

        if not self._confirmBox("Land aircraft %d on approach %s?" % \
                                (item.getID(), approach)):
            return

        # Trigger Land
        wpa = messages.WaypointGoto()
        wpa.msg_dst = int(item.getID())
        wpa.msg_secs = 0
        wpa.msg_nsecs = 0
        if approach == "NEAR":
            wpa.index = 12
        elif approach == "FAR":
            wpa.index = 18
        else:
            print "Invalid landing approach"
            return

        self._sendMessage(wpa)

    def handleLandAbort(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_FLYING])
        if item is None:
            return

        # Trigger Land Abort
        wpa = messages.WaypointGoto()
        wpa.msg_dst = int(item.getID())
        wpa.msg_secs = 0
        wpa.msg_nsecs = 0
        wpa.index = 24

        self._sendMessage(wpa)

    def handleShutdown(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_OFFLINE,
                                     UAVListWidgetItem.STATE_WAITING_AP,
                                     UAVListWidgetItem.STATE_PREFLIGHT,
                                     UAVListWidgetItem.STATE_READY])
        if item is None:
            return

        if not self._confirmBox("Shut down aircraft %d?" % item.getID()):
            return

        # Send a command to shut down the payload
        ps = messages.PayloadShutdown()
        ps.msg_dst = int(item.getID())
        ps.msg_secs = 0
        ps.msg_nsecs = 0
        self._sendMessage(ps)

''' Main code '''

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("flight_tech.py [options]")
    parser.add_option("-d", "--device", dest="device",
                      help="Network device to listen on", default='')
    parser.add_option("-p", "--port", dest="port", type="int",
                      help="Network port to listen on", default=5554)
    parser.add_option("-f", "--flight-tech", dest="ft_mode",
                      action="store_true", default=False,
                      help="Flight Tech mode")
    parser.add_option("-o", "--operator", dest="op_mode",
                      action="store_true", default=False,
                      help="UAV Operator mode")
    parser.add_option("--lo-reverse", dest="lo_reverse",
                      action="store_true", default=False,
                      help="If using lo, reverse the addresses")
    (opts, args) = parser.parse_args()

    # Can constrain which widgets we see based on roles
    show_ft = not opts.op_mode
    show_op = not opts.ft_mode
    if show_ft and show_op:
        print "WARNING: With great power comes great responsibility!"
        mode_string = " -- OMNI MODE"
        ignore = []
    if not show_ft and not show_op:
        print "Please only specify at most one role flag (-f or -o)"
        sys.exit(1)
    if opts.ft_mode:
        mode_string = " -- Flight Tech mode"
        ignore = [UAVListWidgetItem.STATE_FLYING]
    if opts.op_mode:
        mode_string = " -- Operator mode"
        ignore = [UAVListWidgetItem.STATE_WAITING_AP,
                  UAVListWidgetItem.STATE_PREFLIGHT]

    # NOTE: This is a hack to work with SITL
    my_ip = None
    bcast_ip = None
    if opts.device == 'lo':
        my_ip = '127.0.1.1'
        bcast_ip = '127.0.0.1'
        if opts.lo_reverse:
            (my_ip, bcast_ip) = (bcast_ip, my_ip)

    # Establish socket to aircraft
    try:
        sock = Socket(0xff, opts.port, opts.device, my_ip, bcast_ip)
        # NOTE: The next two lines are *definitely* not the most pythonic
        #  (shouldn't just grab class data members)
        my_ip = sock._ip
        bcast_ip = sock._bcast
    except Exception:
        print "Couldn't start up socket on interface '%s'" % opts.device
        sys.exit(1)

    # Build up the window itself
    app = QApplication([])
    win = QWidget()
    win.resize(400, 470 + 120 * int(show_ft and show_op))
    win.setWindowTitle("FTI" + mode_string)
    layout = QVBoxLayout()
    win.setLayout(layout)

    # Status of currently-selected UAV
    # NOTE: The way updates are done is a bit hackish,
    #  relies on a signal being emitted by the UAVListWidget
    lblStat = QLabel("No Aircraft Selected")
    lblStat.setStyleSheet("QLabel { color : black; }")
    lblStat.setAutoFillBackground(True)

    # Listbox of UAVs
    lst = UAVListWidget(sock, ignore, win)

    # Status and IMU lights rows
    laySta = lst.buildStatusLights()
    if show_ft:
        layCfg = lst.buildConfigLights()
        layImu = lst.buildImuLights()

    # Connect list-click to status text and lights
    pltOrig = lblStat.palette()
    def do_update_stat():
        item = lst.currentItem()
        if item is None:
            lblStat.setText("No Aircraft Selected")
            lblStat.setPalette(pltOrig)
        else:
            lblStat.setText(str(item))
            plt = lblStat.palette()
            plt.setBrush(QPalette.Background, item.getState().color)
            lblStat.setPalette(plt)
        lst.updateStatusLights()
        lst.updatePoseLights()
    lst.setSortingEnabled(True)
    lst.itemClicked.connect(do_update_stat)
    lst.itemSelectionChanged.connect(do_update_stat)

    # Add labels and listbox in preferred order
    layout.addWidget(lblStat)
    layout.addLayout(laySta)
    if show_ft:
        layout.addLayout(layCfg)
        layout.addLayout(layImu)
    layout.addWidget(lst)

    # Provide color-key for states
    hlayout = QHBoxLayout()
    for st in [ UAVListWidgetItem.STATE_OFFLINE,
                UAVListWidgetItem.STATE_WAITING_AP,
                UAVListWidgetItem.STATE_PREFLIGHT,
                UAVListWidgetItem.STATE_READY,
                UAVListWidgetItem.STATE_FLYING ]:
        lbl = QLabel(st.text)
        plt = lbl.palette()
        plt.setBrush(QPalette.Background, st.color)
        lbl.setPalette(plt)
        lbl.setAutoFillBackground(True)
        hlayout.addWidget(lbl)
    layout.addLayout(hlayout)

    if show_ft:
        # Mechanical preflight buttons
        mlayout = QHBoxLayout()
        btCalpress = QPushButton("Cal Pressure")
        btCalpress.clicked.connect(lst.handleCalpress)
        mlayout.addWidget(btCalpress)
        btServos = QPushButton("Demo Servos")
        btServos.clicked.connect(lst.handleServos)
        mlayout.addWidget(btServos)
        btMotor = QPushButton("Run Motor")
        btMotor.clicked.connect(lst.handleMotor)
        mlayout.addWidget(btMotor)
        layout.addLayout(mlayout)

    # Arm and disarm throttle buttons
    alayout = QHBoxLayout()
    btArm = QPushButton("ARM Throttle")
    btArm.setStyleSheet("background-color: indianred")
    btArm.clicked.connect(lst.handleArm)
    alayout.addWidget(btArm)
    btDisArm = QPushButton("Disarm Throttle")
    btDisArm.clicked.connect(lst.handleDisArm)
    alayout.addWidget(btDisArm)
    layout.addLayout(alayout)

    if show_ft:
        # Troubleshooting buttons (USE WITH CAUTION)
        tlayout = QHBoxLayout()
        btMAVProxy = QPushButton("Run MAVProxy")
        btMAVProxy.setStyleSheet("background-color: darkgray")
        btMAVProxy.clicked.connect(lst.handleMAVProxy)
        tlayout.addWidget(btMAVProxy)
        btCalgyros = QPushButton("Cal Gyros")
        btCalgyros.setStyleSheet("background-color: darkgray")
        btCalgyros.clicked.connect(lst.handleCalgyros)
        tlayout.addWidget(btCalgyros)
        btAPReboot = QPushButton("Reboot AP")
        btAPReboot.setStyleSheet("background-color: darkgray")
        btAPReboot.clicked.connect(lst.handleAPReboot)
        tlayout.addWidget(btAPReboot)
        layout.addLayout(tlayout)

        # Flight-ready button
        btToggle = QPushButton("Toggle Flight Ready")
        btToggle.clicked.connect(lst.handleFlightReady)
        layout.addWidget(btToggle)

    if show_op:
        # Mode buttons
        mlayout = QHBoxLayout()
        btAUTO = QPushButton("AUTO")
        btAUTO.clicked.connect(lst.handleAUTO)
        mlayout.addWidget(btAUTO)
        lnWP=QLineEdit()
        lnWP.setFixedWidth(50)
        lnWP.setText("1")
        mlayout.addWidget(lnWP)
        btRTL = QPushButton("RTL")
        btRTL.clicked.connect(lst.handleRTL)
        mlayout.addWidget(btRTL)
        layout.addLayout(mlayout)

        # Swarm-ready button
        btSwarmReady = QPushButton("Confirm Swarm Ready")
        btSwarmReady.clicked.connect(lst.handleSwarmReady)
        layout.addWidget(btSwarmReady)

        # Landing buttons
        llayout = QHBoxLayout()
        la_str = "NEAR"
        btLandA = QPushButton("Land " + la_str)
        btLandA.clicked.connect(lambda : lst.handleLand(la_str))
        llayout.addWidget(btLandA)
        lb_str = "FAR"
        btLandB = QPushButton("Land " + lb_str)
        btLandA.clicked.connect(lambda : lst.handleLand(lb_str))
        llayout.addWidget(btLandB)
        btLandAbort = QPushButton("Land Abort")
        btLandAbort.clicked.connect(lst.handleLandAbort)
        llayout.addWidget(btLandAbort)
        layout.addLayout(llayout)

        # Troubleshooting buttons (USE WITH CAUTION)
        tlayout = QHBoxLayout()
        btCalpress = QPushButton("Cal Pressure")
        btCalpress.setStyleSheet("background-color: darkgray")
        btCalpress.clicked.connect(lst.handleCalpress)
        tlayout.addWidget(btCalpress)
        btCalgyros = QPushButton("Cal Gyros")
        btCalgyros.setStyleSheet("background-color: darkgray")
        btCalgyros.clicked.connect(lst.handleCalgyros)
        tlayout.addWidget(btCalgyros)
        btAPReboot = QPushButton("Reboot AP")
        btAPReboot.setStyleSheet("background-color: darkgray")
        btAPReboot.clicked.connect(lst.handleAPReboot)
        tlayout.addWidget(btAPReboot)
        layout.addLayout(tlayout)

    # Shutdown button
    btShutdown = QPushButton("Shut Down Payload")
    btShutdown.clicked.connect(lst.handleShutdown)
    layout.addWidget(btShutdown)

    # Exit button
    btExit = QPushButton("Exit")
    def do_exit():
        sys.exit(0)
    btExit.clicked.connect(do_exit)
    layout.addWidget(btExit)

    # Start the GUI do-loop
    win.show()
    app.exec_()

