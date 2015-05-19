#!/usr/bin/env python

# Super-hackish Flight Tech Interface and Preflight System
# Mike Clement, 2014-2015

# TODO: This is super ugly, no separation of concerns, yada yada
# Redesign so we have a legitimate model-view-controller

from PySide.QtCore import *
from PySide.QtGui import *

import ap_lib.acs_messages as messages
from ap_lib.acs_socket import Socket

import glob
from math import asin, atan2, degrees
from optparse import OptionParser
import os
import subprocess
import sys
import threading
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
    STATE_OFFLINE = UAVState('OFFLINE', QBrush(QColor('darkgray')))
    STATE_WAITING_AP = UAVState('WAITING AP', QBrush(QColor('darkturquoise')))
    STATE_PREFLIGHT = UAVState('PREFLIGHT', QBrush(QColor('yellow')))
    STATE_READY= UAVState('READY', QBrush(QColor.fromRgb(0,200,0)))
    STATE_FLYING = UAVState('FLYING', QBrush(QColor('white')))
    STATE_PROBLEM = UAVState('PROBLEM', QBrush(QColor('red')))

    def __init__(self, *args, **kwargs):
        QListWidgetItem.__init__(self, *args, **kwargs)

        # Attributes of an item
        self._msg_s = None   # Last-received FlightStatus message
        self._msg_p = None   # Last-received Pose message
        self._time = 0.0     # Time last message was received
        self._state = None   # State (color coding)
        self._aspd = []      # History of airspeed readings

        self.setFont(self.FONT)

    # This is used for the top status line
    def __str__(self):
        if self._state is None or self._msg_s is None:
            return "UNKNOWN AIRCRAFT STATUS"
        ap_mode = self.getModeString()
        if ap_mode == "AUTO":
            ap_mode += " / WP %d" % self._msg_s.mis_cur
        return "%d : %s (%s / %2.3fv / %2.1fm)" % \
               (self._msg_s.msg_src,
                self._msg_s.name,
                ap_mode,
                self._msg_s.batt_vcc / 1000.0,
                self._msg_s.alt_rel / 1000.0)

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

    def getReady(self):
        if self._msg_s:
            return self._msg_s.ready
        return False

    def getTime(self):
        return self._time

    def getState(self):
        return self._state

    def getLastStatus(self):
        return self._msg_s

    def getLastPose(self):
        return self._msg_p

    def getAvgAspd(self):
        if self._msg_s is None: return 0
        return float(sum(self._aspd)) / float(len(self._aspd))

    def getMaxAspd(self):
        if self._msg_s is None: return 0
        return float(max(self._aspd))

    def getModeString(self):
        MODES = ['RTL', 'MANUAL', 'FBWA', 'GUIDED', 'AUTO', 'FBWB', 'CIRCLE']
        if self._msg_s is None: return 'UNKNOWN'
        if self.getState() == self.STATE_OFFLINE: return 'OFFLINE'
        if self._msg_s.mode >= len(MODES): return 'UNKNOWN'
        return MODES[self._msg_s.mode]

    # Set the state (color)
    def setState(self, state):
        self._state = state
        if self._state:
            self.setBackground(self._state.color)
        else:
            self.setBackground(self.STATE_NONE.color)
        self.updateText()

    # Update text in item
    def updateText(self):
        status_text = self.__str__()
        if self._msg_s and self._msg_s.armed:
            status_text += " <ARMED>"
        self.setText(status_text)

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
        if msg.ready and msg.alt_rel > 10000:
            # Ready && (Alt > 10m AGL) -> Active/Flying
            # NOTE: ok conditions should be integrated elsewhere
            # NOTE: no airspeed integrated yet (tricky for takeoff)
            all_ok = msg.armed and msg.ok_ahrs and msg.ok_as and \
                     msg.ok_gps and msg.ok_ins and msg.ok_mag and \
                     msg.ok_pwr and msg.ready and \
                     (msg.mode == 4) and \
                     (msg.batt_vcc > 10800)
                     # TODO: Add airspeed watch (for stall)
            if all_ok:
                state = self.STATE_FLYING
            else:
                state = self.STATE_PROBLEM
        elif msg.batt_vcc == 0.0 and msg.mode == 15:
            # No voltage ^ Unknown mode -> No autopilot data yet
            state = self.STATE_WAITING_AP
        elif msg.ready:
            state = self.STATE_READY
        else:
            state = self.STATE_PREFLIGHT
        self.setState(state)

        # Update text for item
        self.updateText()

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
        self.mav_popen_lock = threading.Lock()
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

        if 'udp:' in self.mav_channel:
            ss = messages.SlaveSetup()
            ss.msg_dst = int(self.mav_id)
            ss.msg_secs = 0
            ss.msg_nsecs = 0
            ss.enable = False
            ss.channel = self.mav_channel
            self._sendMessage(ss)

        self.mav_id = None
        self.mav_channel = None

    ''' Status and IMU lights '''

    class _QLight(QLabel):

        def __init__(self, name, tip=''):
            QLabel.__init__(self, name)
            self._name = name
            self._tip = tip
            if tip: self.setToolTip(tip)

        def unknown(self):
            self.setStyleSheet("QLabel { color : black; }")
            self.setToolTip(self._tip)

        def bad(self, text='', tip=''):
            self.setStyleSheet("QLabel { color : red; font: bold; }")
            self.setText(self._name + text)
            if tip: self.setToolTip(tip)
            else: self.setToolTip(self._tip)

        def ok(self, text='', tip=''):
            self.setStyleSheet("QLabel { color : orange; font: bold; }")
            self.setText(self._name + text)
            if tip: self.setToolTip(tip)
            else: self.setToolTip(self._tip)

        def good(self):
            self.setStyleSheet("QLabel { color : green; }")
            self.setText(self._name)
            self.setToolTip(self._tip)

        def setbool(self, b_good, b_ok=False,
                    oktext='', oktip='',
                    badtext='', badtip=''):
            if b_good: self.good()
            elif b_ok: self.ok(text=oktext, tip=oktip)
            else: self.bad(text=badtext, tip=badtip)

    def _buildLights(self, light_list):
        layout = QHBoxLayout()
        for l in light_list:
            lbl = self._QLight(name = l[0], tip = l[1])
            self.lights[l[0]] = lbl
            layout.addWidget(lbl)
        return layout

    LIGHT_STA_OK = [('ahrs', ''),
                    ('as', ''),
                    ('gps', ''),
                    ('ins', ''),
                    ('mag', ''),
                    ('pwr', '')]
    LIGHT_STA_CP = [('arm', ''),
                    ('aspd', ''),
                    ('batt', ''),
                    ('ralt', '')]
    LIGHT_STA = sorted(LIGHT_STA_OK + LIGHT_STA_CP)
    LIGHT_CFG_OK = [('ral', ''),
                    ('wp', ''),
                    ('fen', ''),
                    ('prm', '')]
    LIGHT_CFG_CP = [('fw', ''),
                    ('sw', ''),
                    ('repo', '')]
    LIGHT_CFG = LIGHT_CFG_CP + LIGHT_CFG_OK
    LIGHT_IMU = [('UP', ''),
                 ('DOWN', ''),
                 ('LEFT', ''),
                 ('RIGHT', '')]

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
        for l in self.LIGHT_STA_OK + self.LIGHT_CFG_OK:
            light = l[0]
            badtip = ''
            if light not in self.lights:
                continue

            # Set custom tooltips for some lights
            if light == 'as':
                badtip = 'bad airspeed (try recalibrating)'
            elif light in [c[0] for c in self.LIGHT_CFG_OK]:
                badtip = 'verification not completed'
            else:
                badtip = 'bad sensor health'

            # Update the light
            self.lights[light].setbool(getattr(msg, 'ok_'+light),
                                       badtip=badtip)

        # Update remaining (computed) lights

        # Arming status
        if item.getState() in [UAVListWidgetItem.STATE_FLYING,
                               UAVListWidgetItem.STATE_PROBLEM]:
            # If flying, being disarmed is BAD
            self.lights['arm'].setbool(msg.armed,
                                       badtip='Disarmed in flight')
        elif item.getReady():
            # If flight ready, we *want* to be armed (but no error if not)
            self.lights['arm'].setbool(msg.armed, True,
                                       oktip='Warning: arm before launching')
        else:
            # Otherwise, we want to warn tech while armed
            self.lights['arm'].setbool(not msg.armed, True,
                                       oktip='Warning: armed during preflight')

        # Relative alt must be +/- 10 m
        if item.getState() in [UAVListWidgetItem.STATE_FLYING,
                               UAVListWidgetItem.STATE_PROBLEM]:
            # Ignore if flying
            self.lights['ralt'].good()
        else:
            # Make sure within calibration tolerances
            self.lights['ralt'].setbool(-10000.0 < msg.alt_rel < 10000.0,
                                        badtip='Out of tolerance (> +/-10 m)')

        # Average airspeed (over 10 samples) must have *some* deviation,
        # but not too much
        as_avg = item.getAvgAspd()
        # Max airspeed used so we can check positive pressure on the ground
        as_max = item.getMaxAspd()
        if item.getState() in [UAVListWidgetItem.STATE_FLYING,
                               UAVListWidgetItem.STATE_PROBLEM]:
            # If flying, make sure above stall speed
            # TODO: This condition "flickers" on takeoff, find better logic
            self.lights['aspd'].setbool(12.0 < as_avg, badtext='/S',
                                        badtip='Stall warning (< 12 m/s)')
        elif item.getState() == UAVListWidgetItem.STATE_READY:
            # If ready, only check for flatline
            self.lights['aspd'].setbool(0.0 < as_avg, badtext='/F',
                                        badtip='Flatline (0 m/s)')
        elif as_max > 20.0:
            # On ground, spike (hopefully) means checking positive pressure
            self.lights['aspd'].ok(text='/P',
                                   tip='Positive pressure check (> 20 m/s)')
        elif as_avg == 0.0:
            # On ground, flatline is bad
            self.lights['aspd'].bad(text='/F',
                                    tip='Flatline (0 m/s)')
        else:
            # On ground, should have minimal, nonzero noise
            self.lights['aspd'].setbool(0.0 < as_avg < 5.0, badtext='/T',
                                        badtip='Out of tolerance (> 5 m/s)')

        # Battery voltage should be > 12.4 V at takeoff
        # using "ok" color to distinguish from actual problem
        vcc = msg.batt_vcc
        if item.getState() in [UAVListWidgetItem.STATE_FLYING,
                               UAVListWidgetItem.STATE_PROBLEM]:
            self.lights['batt'].setbool(vcc > 10800,
                                        badtip='Low battery (< 10.8 V)')
        else:
            self.lights['batt'].setbool(vcc > 12000, vcc > 10800,
                                        oktip='Low for takeoff (< 12.0 V)',
                                        badtip='Low battery (< 10.8 V)')

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
        if self.mav_popen not in [None, False]:
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

        # Hide those we haven't seen in a while or that we've filtered
        cur_time = time.time()
        for i in range(self.count()):
            if not self.item(i).getID():
                # Ignore the "none" item
                continue
            if self.item(i).getState() in self.filter_states:
                self.item(i).setHidden(True)
            elif self.item(i).getTime() < (cur_time - self.DELETE_TIME):
                # Flying (problem) aircraft shouldn't disappear
                if self.item(i).getState() != UAVListWidgetItem.STATE_PROBLEM:
                    self.item(i).setHidden(True)
            elif self.item(i).getTime() < (cur_time - self.OFFLINE_TIME):
                # If flying, this is a problem. Otherwise, it's ok
                if self.item(i).getState() in [UAVListWidgetItem.STATE_FLYING,
                                               UAVListWidgetItem.STATE_PROBLEM]:
                    self.item(i).setState(UAVListWidgetItem.STATE_PROBLEM)
                else:
                    self.item(i).setState(UAVListWidgetItem.STATE_OFFLINE)
            else:
                self.item(i).setHidden(False)

        # If selected item becomes hidden, de-select it
        if self.currentItem() and self.currentItem().isHidden():
            self.setCurrentItem(self.dummy)

        # schedule refresh and raise event
        self.update()
        self.itemSelectionChanged.emit()

    ''' Task handlers '''

    def handleMAVProxy(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_WAITING_AP,
                                     UAVListWidgetItem.STATE_PREFLIGHT,
                                     UAVListWidgetItem.STATE_READY,
                                     UAVListWidgetItem.STATE_FLYING,
                                     UAVListWidgetItem.STATE_PROBLEM])
        if item is None:
            return

        # Can only support one MAVProxy subprocess (for now)
        with self.mav_popen_lock:
            if self.mav_popen is not None:
                print "Please close your other MAVProxy instance first"
                return

            # Set to something other than None for now
            self.mav_popen = False

        # If we get here, start a thread to continue launching
        self.mav_id = int(item.getID())
        t = threading.Thread(target=self.doMAVProxy)
        t.daemon = True
        t.start()

    # Continues the above handler in a separate thread
    def doMAVProxy(self):
        # See if we have a telem radio that we can configure
        use_telem = True
        radios = glob.glob("/dev/serial/by-id/usb-FTDI*")
        if len(radios) < 1:
            print "No telem radios found, using network ..."
            use_telem = False
        else:
            try:
                # NOTE: SUPER DUPER HACKISH IMPORT!!!
                # Possible that SiK tools aren't installed!
                atc_path = "$ACS_ROOT/SiK/Firmware/tools/"
                sys.path.append(os.path.expandvars(atc_path))
                from atcommander import ATCommandSet
                print "Trying %s ..." % radios[0]
                atc = ATCommandSet(radios[0])
            except Exception as ex:
                print "ATCommandSet: " + str(ex)
                use_telem = False

        # If we have a radio and have atcommander.py, try configuration
        if use_telem:
            # Enumerate commands by function call
            commands = [lambda: atc.leave_command_mode_force(),
                        lambda: atc.unstick(),
                        lambda: atc.enter_command_mode(),
                        lambda: atc.set_param(ATCommandSet.PARAM_NETID,
                                              self.mav_id),
                        lambda: atc.write_params(),
                        lambda: atc.reboot(),
                        lambda: atc.leave_command_mode()]

            # Iterate through commands, trying a bounded number of times overall
            retries_left = 10
            command = commands.pop(0)
            while retries_left > 0:
                try:
                    # Always give a little time for the radio to settle
                    time.sleep(0.1)
                    res = command()
                    if res == False:
                        # Some commands return True/False, others return None
                        # Only False is bad
                        raise Exception("command failed")
                    if len(commands) == 0:
                        # If no commands left, we're done
                        break
                    command = commands.pop(0)
                except Exception as ex:
                    print "Config exception: " + str(ex) + ", retrying ..."
                    retries_left -= 1

            # Handle success or failure
            if retries_left > 0:
                self.mav_channel = radios[0]
            else:
                print "Falling back to network ..."
                use_telem = False

        # If using the network, set up slave channel
        if not use_telem:
            # Can't MAVProxy to a WAITING AP aircraft via network!
            if self.itemByIdent(self.mav_id).getState() == \
               UAVListWidgetItem.STATE_WAITING_AP:
                print "Cannot MAVProxy by network to WAITING AP plane"
                self.mav_popen = None
                return

            # Pick an aircraft-unique port
            slave_port = 15554 + self.mav_id
            self.mav_channel = "udp:%s:%u" % (self._sock._ip, slave_port)

            # Open a slave mavlink channel to the aircraft
            # NOTE: This is done unreliably, so it might fail and we won't know
            ss = messages.SlaveSetup()
            ss.msg_dst = self.mav_id
            ss.msg_secs = 0
            ss.msg_nsecs = 0
            ss.enable = True
            ss.channel = self.mav_channel
            self._sendMessage(ss)

        # Wait a moment for the radio/payload to catch up
        time.sleep(0.5)

        # Start up MAVProxy instance
        print "Starting MAVProxy session ..."
        try:
            self.mav_start_time = time.time()
            self.mav_popen = subprocess.Popen( \
                "xterm -e mavproxy.py --master %s --baudrate 57600" % \
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

        if not self._confirmBox("WARNING: Run motor for aircraft %d?" % \
                                item.getID()):
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

    def handleConfig(self, stack, alt):
        item = self._checkItemState([UAVListWidgetItem.STATE_READY])
        if item is None:
            return

        try:
            mc_alt = int(alt)
            if mc_alt <= 50:
                raise Exception("")
            mc_stack = int(stack)
            if mc_stack <= 0:
                raise Exception("")
        except:
            print "You must supply a valid altitude!"
            return

        if not self._confirmBox("Send config to aircraft %d?" % item.getID()):
            return

        # Send desired mission config
        mc = messages.MissionConfig()
        mc.msg_dst = int(item.getID())
        mc.msg_secs = 0
        mc.msg_nsecs = 0
        mc.std_alt = mc_alt
        mc.stack_num = mc_stack
        self._sendMessage(mc)

    def handleAUTO(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_READY,
                                     UAVListWidgetItem.STATE_FLYING,
                                     UAVListWidgetItem.STATE_PROBLEM])
        if item is None:
            return

        # Trigger AUTO
        au = messages.Mode()
        au.msg_dst = int(item.getID())
        au.msg_secs = 0
        au.msg_nsecs = 0
        au.mode = 4           # AUTO mode number

        self._sendMessage(au)

    def handleWP(self, index):
        item = self._checkItemState([UAVListWidgetItem.STATE_READY,
                                     UAVListWidgetItem.STATE_FLYING,
                                     UAVListWidgetItem.STATE_PROBLEM])
        if item is None:
            return

        # Get and set desired WP #
        try:
            wp_des = int(index)
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

    def handleRTL(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_FLYING,
                                     UAVListWidgetItem.STATE_PROBLEM])
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
        item = self._checkItemState([UAVListWidgetItem.STATE_FLYING,
                                     UAVListWidgetItem.STATE_PROBLEM])
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
        if approach == "A":
            wpa.index = 12     # TODO: Move index selection to payload
        elif approach == "B":
            wpa.index = 18     # TODO: Move index selection to payload
        else:
            print "Invalid landing approach"
            return

        self._sendMessage(wpa)

        # Make sure aircraft is in AUTO as well
        self.handleAUTO()

    def handleLandAbort(self):
        item = self._checkItemState([UAVListWidgetItem.STATE_FLYING,
                                     UAVListWidgetItem.STATE_PROBLEM])
        if item is None:
            return

        # Trigger Land Abort
        wpa = messages.WaypointGoto()
        wpa.msg_dst = int(item.getID())
        wpa.msg_secs = 0
        wpa.msg_nsecs = 0
        wpa.index = 24     # TODO: Move index selection to payload

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

''' Helper functions '''

# Add a horizontal separator
def addLine(layout):
    l = QFrame()
    l.setFrameShape(QFrame.HLine)
    l.setFrameShadow(QFrame.Sunken)
    layout.addWidget(l)

''' Main code '''

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("flight_tech.py [options]")
    parser.add_option("-i", "--id", dest="id", type="int",
                      help="ID to use for sending commands", default=255)
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
    # NOTE: This is an old SITL hack; can probably remove
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
        ignore = [UAVListWidgetItem.STATE_FLYING,
                  UAVListWidgetItem.STATE_PROBLEM]
    if opts.op_mode:
        mode_string = " -- Operator mode"
        ignore = [UAVListWidgetItem.STATE_OFFLINE,
                  UAVListWidgetItem.STATE_WAITING_AP,
                  UAVListWidgetItem.STATE_PREFLIGHT]

    # NOTE: This is an old hack for SITL, and can probably be removed unless
    # we're planning to use the loopback interface again in the future.
    my_ip = None
    bcast_ip = None
    if opts.device == 'lo':
        my_ip = '127.0.1.1'
        bcast_ip = '127.0.0.1'
        if opts.lo_reverse:     # NOTE: also safe to remove this logic
            (my_ip, bcast_ip) = (bcast_ip, my_ip)

    # Establish socket to aircraft
    try:
        sock = Socket(opts.id, opts.port, opts.device, my_ip, bcast_ip)
        # NOTE: The next two lines are *definitely* not the most pythonic
        #  (shouldn't just grab class data members)
        my_ip = sock._ip
        bcast_ip = sock._bcast
    except Exception:
        print "Couldn't start up socket on interface '%s'" % opts.device
        sys.exit(1)

    # Build up the main window
    app = QApplication([])
    win = QWidget()
    win.resize(1000, 670)
    win.setWindowTitle("FTI" + mode_string)
    lay_main = QHBoxLayout()
    win.setLayout(lay_main)
    lay_left = QVBoxLayout()   # Left side of window
    lay_main.addLayout(lay_left)
    lay_right = QVBoxLayout()  # Right side of window
    lay_main.addLayout(lay_right)

    # Enforce minimum sizes for two panels
    # (Based on experimentation, these sizes seem to display all text)
    lay_left.addStrut(550)
    lay_right.addStrut(450)

    # Status of currently-selected UAV
    # NOTE: The way updates are done is a bit hackish,
    #  relies on a signal being emitted by the UAVListWidget
    lblStat = QLabel("No Aircraft Selected")
    lblStat.setStyleSheet("QLabel { color : black; font-size : 20px; }")
    lblStat.setAutoFillBackground(True)

    # Listbox of UAVs
    lst = UAVListWidget(sock, ignore, win)

    # Status and IMU lights rows
    laySta = lst.buildStatusLights()
    layCfg = lst.buildConfigLights()
    if show_ft:
        layImu = lst.buildImuLights()

    # Connect list-click to status text and lights
    pltOrig = lblStat.palette()
    def do_update_stat():
        item = lst.currentItem()
        if item is None:
            lblStat.setText("No Aircraft Selected")
            lblStat.setPalette(pltOrig)
            lblStat.setToolTip('')
        else:
            lblStat.setText(str(item))
            plt = lblStat.palette()
            plt.setBrush(QPalette.Background, item.getState().color)
            lblStat.setPalette(plt)
            lblStat.setToolTip(item.getIP())
        # Need to call these to refresh status lights when selection changes
        lst.updateStatusLights()
        lst.updatePoseLights()
    lst.setSortingEnabled(True)
    lst.itemClicked.connect(do_update_stat)
    lst.itemSelectionChanged.connect(do_update_stat)

    # Place listbox and color key
    lay_left.addWidget(lst)
    hlayout = QHBoxLayout()
    for st in [ UAVListWidgetItem.STATE_OFFLINE,
                UAVListWidgetItem.STATE_WAITING_AP,
                UAVListWidgetItem.STATE_PREFLIGHT,
                UAVListWidgetItem.STATE_READY,
                UAVListWidgetItem.STATE_FLYING,
                UAVListWidgetItem.STATE_PROBLEM]:
        lbl = QLabel(st.text)
        plt = lbl.palette()
        plt.setBrush(QPalette.Background, st.color)
        lbl.setPalette(plt)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setAutoFillBackground(True)
        hlayout.addWidget(lbl)
    lay_left.addLayout(hlayout)

    # Place status rows
    lay_right.addWidget(lblStat)
    lay_right.addLayout(laySta)
    lay_right.addLayout(layCfg)
    if show_ft:
        lay_right.addLayout(layImu)
    addLine(lay_right)

    # Mission config
    if show_op:
        lbl = QLabel("MISSION CONFIGURATION")
        lbl.setAlignment(Qt.AlignCenter)
        lay_right.addWidget(lbl)
        # Set up layouts
        clayout = QHBoxLayout()
        clay_left = QVBoxLayout()
        clay_left.addStrut(270)
        clay_right = QVBoxLayout()
        # Stack number
        clay_left_row = QHBoxLayout()
        lbl = QLabel("Stack number:")
        clay_left_row.addWidget(lbl)
        lnStack = QLineEdit()
        lnStack.setFixedWidth(50)
        lnStack.setAlignment(Qt.AlignRight)
        lnStack.setText("1")
        clay_left_row.addWidget(lnStack)
        clay_left.addLayout(clay_left_row)
        # Standard altitude
        clay_left_row = QHBoxLayout()
        lbl = QLabel("Altitude above runway (m):")
        clay_left_row.addWidget(lbl)
        lnAlt = QLineEdit()
        lnAlt.setFixedWidth(50)
        lnAlt.setAlignment(Qt.AlignRight)
        lnAlt.setText("100")
        clay_left_row.addWidget(lnAlt)
        clay_left.addLayout(clay_left_row)
        # Populate left side
        clayout.addLayout(clay_left)
        # Send button
        btConfig = QPushButton("Send Config")
        btConfig.clicked.connect(lambda : lst.handleConfig(lnStack.text(),
                                                           lnAlt.text()))
        clay_right.addWidget(btConfig)
        # Populate right side
        clayout.addStretch(1)
        clayout.addLayout(clay_right)
        # Add to main layout
        lay_right.addLayout(clayout)

        # Line break after config stuff
        addLine(lay_right)

    lbl = QLabel("PRIMARY FUNCTIONS")
    lbl.setAlignment(Qt.AlignCenter)
    lay_right.addWidget(lbl)

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
        lay_right.addLayout(mlayout)

    # Arm and disarm throttle buttons
    alayout = QHBoxLayout()
    btArm = QPushButton("ARM Throttle")
    btArm.setStyleSheet("background-color: indianred")
    btArm.clicked.connect(lst.handleArm)
    alayout.addWidget(btArm)
    btDisArm = QPushButton("Disarm Throttle")
    btDisArm.clicked.connect(lst.handleDisArm)
    alayout.addWidget(btDisArm)
    lay_right.addLayout(alayout)

    if show_ft:
        # Flight-ready button
        btToggle = QPushButton("Toggle Flight Ready")
        btToggle.clicked.connect(lst.handleFlightReady)
        lay_right.addWidget(btToggle)

    if show_op:
        # Mode buttons
        mlayout = QHBoxLayout()
        btAUTO = QPushButton("AUTO")
        btAUTO.clicked.connect(lst.handleAUTO)
        mlayout.addWidget(btAUTO)
        btWP = QPushButton("Send WP")
        mlayout.addWidget(btWP)
        lnWP=QLineEdit()
        lnWP.setFixedWidth(50)
        lnWP.setText("1")
        mlayout.addWidget(lnWP)
        btWP.clicked.connect(lambda : lst.handleWP(lnWP.text()))
        btRTL = QPushButton("RTL")
        btRTL.setStyleSheet("background-color: indianred")
        btRTL.clicked.connect(lst.handleRTL)
        mlayout.addWidget(btRTL)
        lay_right.addLayout(mlayout)

        # Landing buttons
        llayout = QHBoxLayout()
        la_str = "A"
        btLandA = QPushButton("Land " + la_str)
        btLandA.clicked.connect(lambda : lst.handleLand(la_str))
        llayout.addWidget(btLandA)
        lb_str = "B"
        btLandB = QPushButton("Land " + lb_str)
        btLandB.clicked.connect(lambda : lst.handleLand(lb_str))
        llayout.addWidget(btLandB)
        btLandAbort = QPushButton("Land Abort")
        btLandAbort.clicked.connect(lst.handleLandAbort)
        llayout.addWidget(btLandAbort)
        lay_right.addLayout(llayout)

    # Line break before troubleshooting and misc buttons
    addLine(lay_right)

    lbl = QLabel("UTILITY FUNCTIONS")
    lbl.setAlignment(Qt.AlignCenter)
    lay_right.addWidget(lbl)

    # Troubleshooting buttons (USE WITH CAUTION)
    if show_op and not show_ft:
        btMAVProxy = QPushButton("Run MAVProxy")
        btMAVProxy.setStyleSheet("background-color: darkgray")
        btMAVProxy.clicked.connect(lst.handleMAVProxy)
        lay_right.addWidget(btMAVProxy)

    tlayout = QHBoxLayout()
    if show_ft:
        btMAVProxy = QPushButton("Run MAVProxy")
        btMAVProxy.setStyleSheet("background-color: darkgray")
        btMAVProxy.clicked.connect(lst.handleMAVProxy)
        tlayout.addWidget(btMAVProxy)
    elif show_op:
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
    lay_right.addLayout(tlayout)

    # Shutdown button
    btShutdown = QPushButton("Shut Down Payload")
    btShutdown.clicked.connect(lst.handleShutdown)
    lay_right.addWidget(btShutdown)

    # Allow blank space below buttons
    lay_right.addStretch(1)

    # Exit button
    btExit = QPushButton("Exit")
    def do_exit():
        sys.exit(0)
    btExit.clicked.connect(do_exit)
    lay_right.addWidget(btExit)

    # Start the GUI do-loop
    win.show()
    app.exec_()

