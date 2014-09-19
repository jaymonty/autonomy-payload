#!/usr/bin/env python

from PySide.QtCore import *
from PySide.QtGui import *

from ap_lib import acs_messages
from ap_lib.acs_socket import Socket

from optparse import OptionParser
import subprocess
import sys
import time

# Simple representation of UAV state
class UAVState():
    def __init__(self, text, color):
        self.text = text
        self.color = color

# A QListWidgetItem for UAVs, with some custom data and printing
class UAVListWidgetItem(QListWidgetItem):
    FONT = QFont('Helvetica', 14)

    def __init__(self, ident, name, ip, state):
        QListWidgetItem.__init__(self)
        self.ident = ident
        self.name = name
        self.ip = ip

        self.setState(state)
        self.setVoltage(0.0)
        self.updateLastSeen()

        self.setFont(UAVListWidgetItem.FONT)
        self.setText("%s (%d / %2.3fv)" % (self.name, self.ident, self.vcc))

    def __str__(self):
        return "%s : %s (%d / %s / %2.3fv)" % \
               (self.name, self.state.text, self.ident, self.ip, self.vcc)

    def setState(self, state):
        self.state = state
        self.setBackground(self.state.color)

    def setVoltage(self, vcc):
        self.vcc = vcc

    def updateLastSeen(self):
        self.last_seen = time.time()

# This creates a list box populated by listening to ACS messages
class UAVListWidget(QListWidget):

    # UAV state
    STATE_NONE = UAVState('INVALID', QBrush(QColor('white')))
    STATE_OFFLINE = UAVState('OFFLINE', QBrush(QColor('red')))
    STATE_NOT_READY = UAVState('NOT READY', QBrush(QColor('yellow')))
    STATE_READY= UAVState('READY', QBrush(QColor.fromRgb(0,200,0)))
    STATE_ACTIVE = UAVState('ACTIVE', QBrush(QColor.fromRgb(160,160,160)))

    # Time (in seconds) until UAVs go to "offline" status
    OFFLINE_TIME = 5.0

    def __init__(self, sock, parent=None):
        QListWidget.__init__(self, parent)

        # Track UAVs, periodically checking for new messages
        self.uav_listen_event = self.startTimer(1000)

        # Add a dummy aircraft to "select none"
        self.addItem(UAVListWidgetItem(0, '(None)', '', UAVListWidget.STATE_NONE))

        # ACS protocol connection
        self.sock = sock

    # Return the *singly-selected* list item
    def currentItem(self):
        sel = self.selectedItems()
        if len(sel) != 1 or sel[0].ident == 0:
            return None
        return sel[0]

    # Return item matching a given ident
    def itemByIdent(self, ident):
        for i in range(self.count()):
            if self.item(i).ident == ident:
                return self.item(i)
        return None

    # Periodic event to look for new UAVs and to update the list
    def timerEvent(self, event):
        if event.timerId() != self.uav_listen_event:
            return

        # Current time, used for staling out old reports
        cur_time = time.time()

        while True:
            # Try to receive a message
            msg = self.sock.recv()

            # Break if no message, skip non-FlightStatus messages
            if msg is None:
                break
            if not isinstance(msg, acs_messages.FlightStatus):
                continue

            # Color code based on status
            if msg.armed and msg.alt_rel > 20000:
                # Armed ^ (Alt > 20m AGL) -> Active/Flying
                state = UAVListWidget.STATE_ACTIVE
            elif msg.ready:
                state = UAVListWidget.STATE_READY
            else:
                state = UAVListWidget.STATE_NOT_READY

            # Create if not already in the list
            item = self.itemByIdent(msg.msg_src)
            if item is None:
                self.addItem(UAVListWidgetItem(msg.msg_src, msg.name, msg.msg_src_ip, state))
            else:
                item.setState(state)
                item.setVoltage(float(msg.batt_vcc) / 1e03)
                item.updateLastSeen()

        # If we haven't seen any for a while, change color
        for i in range(self.count()):
            if self.item(i).ident == 0:
                continue
            if self.item(i).last_seen < (cur_time - UAVListWidget.OFFLINE_TIME):
                self.item(i).setState(UAVListWidget.STATE_OFFLINE)

        # schedule refresh and raise event
        self.update()
        self.itemSelectionChanged.emit()

def preflight_aircraft(sock, uavid, localip, uavip):
    # Open a slave mavlink channel to the aircraft
    # NOTE: This is done unreliably, so it might fail and we won't know :(
    ss = acs_messages.SlaveSetup()
    ss.msg_dst = int(uavid)
    ss.msg_secs = 0
    ss.msg_nsecs = 0
    ss.enable = True
    ss.channel = "udp:%s:5556" % localip
    sock.send(ss)

    time.sleep(1)

    # Start up a MAVProxy instance and connect to slave channel
    res = subprocess.call("xterm -e mavproxy.py --master %s --load-module preflight" % ss.channel, shell=True)

    # Shut down slave channel
    ss.enable = False
    sock.send(ss)

def set_aircraft_ready(sock, uavid, localip, uavip, ready):
    # Toggle the aircraft to be flight-ready, or not, based on 'uavstat'
    fr = acs_messages.FlightReady()
    fr.msg_dst = int(uavid)
    fr.msg_secs = 0
    fr.msg_nsecs = 0
    fr.ready = ready
    sock.send(fr)

def shutdown_aircraft(sock, uavid, localip, uavip):
    # Send a command to shut down the payload
    ps = acs_messages.PayloadShutdown()
    ps.msg_dst = int(uavid)
    ps.msg_secs = 0
    ps.msg_nsecs = 0
    sock.send(ps)

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("flight_tech.py [options]")
    parser.add_option("--device", dest="device",
                      help="Network device to listen on", default='')
    parser.add_option("--port", dest="port", type="int",
                      help="Network port to listen on", default=5554)
    (opts, args) = parser.parse_args()

    # NOTE: This is a hack to work with SITL
    my_ip = None
    bcast_ip = None
    if opts.device == 'lo':
        my_ip = '127.0.1.1'
        bcast_ip = '127.0.0.1'

    # Establish socket to aircraft
    try:
        sock = Socket(0xff, opts.port, opts.device, my_ip, bcast_ip)
        # NOTE: The next two lines are probably not the most pythonic
        #  (shouldn't just grab class data members)
        my_ip = sock.my_ip
        bcast_ip = sock.bcast_ip
    except Exception:
        print "Couldn't start up socket on interface '%s'" % opts.device
        sys.exit(1)

    # Build up the window itself
    app = QApplication([])
    win = QWidget()
    win.resize(300, 400)
    layout = QVBoxLayout()
    win.setLayout(layout)

    # Listbox of UAVs
    lst = UAVListWidget(sock, win)
    layout.addWidget(lst)

    # Status of currently-selected UAV
    # NOTE: The way updates are done is a bit hackish,
    #  relies on a signal being emitted by the UAVListWidget
    lblStat = QLabel("No Aircraft Selected")
    def do_update_stat():
        if lst.currentItem() is None:
            lblStat.setText("No Aircraft Selected")
        else:
            lblStat.setText(str(lst.currentItem()))
    lst.setSortingEnabled(True)
    lst.itemClicked.connect(do_update_stat)
    lst.itemSelectionChanged.connect(do_update_stat)
    layout.addWidget(lblStat)

    # Pre-flight button
    btPreflight = QPushButton("Pre-Flight")
    def do_preflight():
        item = lst.currentItem()
        if item.ident > 0 and item.state in [UAVListWidget.STATE_READY,
                                            UAVListWidget.STATE_NOT_READY]:
            print "START  %f" % time.time()
            preflight_aircraft(sock, item.ident, my_ip, item.ip)
            print "STOP   %f" % time.time()
    btPreflight.clicked.connect(do_preflight)
    layout.addWidget(btPreflight)

    # Flight-ready button
    btToggle = QPushButton("Toggle Flight Ready")
    def do_toggle():
        item = lst.currentItem()
        if item.ident <= 0:
            return
        if item.state == UAVListWidget.STATE_NOT_READY:
            set_aircraft_ready(sock, item.ident, my_ip, item.ip, True)
        elif item.state == UAVListWidget.STATE_READY:
            set_aircraft_ready(sock, item.ident, my_ip, item.ip, False)
    btToggle.clicked.connect(do_toggle)
    layout.addWidget(btToggle)

    # Shutdown button
    btShutdown = QPushButton("Shut Down Payload")
    def do_shutdown():
        item = lst.currentItem()
        if item.ident <= 0 or item.state == UAVListWidget.STATE_ACTIVE:
            return
        mbx = QMessageBox()
        mbx.setText("Shut down aircraft %d?" % item.ident)
        mbx.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        mbx.setDefaultButton(QMessageBox.No)
        if mbx.exec_() == QMessageBox.Yes:
            shutdown_aircraft(sock, item.ident, my_ip, item.ip)
    btShutdown.clicked.connect(do_shutdown)
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

