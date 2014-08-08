#!/usr/bin/env python

from PySide.QtCore import *
from PySide.QtGui import *

from acs import acs_messages
from acs.acs_socket import Socket

from optparse import OptionParser
import subprocess
import sys
import time

# This creates a list box populated by listening to ACS messages
class UAVListWidget(QListWidget):

    # Some parameters for how things are displayed
    OFFLINE_TIME = 5.0
    STATE_OFFLINE = QBrush(QColor('red'))
    STATE_NOT_READY= QBrush(QColor('yellow'))
    STATE_READY = QBrush(QColor('green'))
    LIST_FONT = QFont('Helvetica', 14)
    # TODO: self.LIST_ICON = QIcon('aircraft.png')

    def __init__(self, sock, parent=None):
        QListWidget.__init__(self, parent)

        # Track UAVs, periodically checking for new messages
        self.uav_list = {}
        self.uav_listen_event = self.startTimer(1000)

        # ACS protocol connection
        self.sock = sock

    # Return the number and status of the currently selected aircraft
    def currentItemInfo(self):
        sel = self.selectedItems()
        if len(sel) != 1:
            return (-1, '', '')
        selid = sel[0].text()
        return (selid, self.uav_list[int(selid)]['i'], self.uav_list[int(selid)]['s'])

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

            # Create if not already in the list
            if msg.msg_src not in self.uav_list:
                self.uav_list[msg.msg_src] = { 'q' : QListWidgetItem(self) }
                self.uav_list[msg.msg_src]['q'].setFont(UAVListWidget.LIST_FONT)
                self.uav_list[msg.msg_src]['q'].setText(str(msg.msg_src))
                self.uav_list[msg.msg_src]['i'] = msg.msg_src_ip
                # TODO: self.uav_list[msg.msg_src]['q'].setIcon(self.LIST_ICON)

            # Color code based on status
            if msg.ready:
                self.uav_list[msg.msg_src]['q'].setBackground(UAVListWidget.STATE_READY)
                self.uav_list[msg.msg_src]['s'] = "READY"
            else:
                self.uav_list[msg.msg_src]['q'].setBackground(UAVListWidget.STATE_NOT_READY)
                self.uav_list[msg.msg_src]['s'] = "NOT READY"

            # Update last-seen timestamp
            self.uav_list[msg.msg_src]['t'] = cur_time

        # If we haven't seen any for a while, change color
        for i in self.uav_list:
            if self.uav_list[i]['t'] < (cur_time - UAVListWidget.OFFLINE_TIME):
                self.uav_list[i]['q'].setBackground(UAVListWidget.STATE_OFFLINE)
                self.uav_list[i]['s'] = "OFFLINE"

        # TODO: Implement sorting, first by status then by ID?

        # schedule refresh and raise event
        self.update()
        self.itemSelectionChanged.emit()

def prep_aircraft(sock, uavid, localip, uavip):
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

def toggle_aircraft(sock, uavid, localip, uavip, uavstat):
    # Toggle the aircraft to be flight-ready, or not, based on 'uavstat'
    fr = acs_messages.FlightReady()
    fr.msg_dst = int(uavid)
    fr.msg_secs = 0
    fr.msg_nsecs = 0
    if uavstat == "NOT READY":  # TODO don't hard-code these
        fr.ready = True
    elif uavstat == "READY":
        fr.ready = False
    else:  # if STATE_OFFLINE or something else, don't do anything
        return
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
    win.resize(280, 400)
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
        (uavid, uavip, uavstat) = lst.currentItemInfo()
        if uavid > 0:
            lblStat.setText("Aircraft %d (%s): %s" % (int(uavid), uavip, uavstat))
        else:
            lblStat.setText("No Aircraft Selected")
    lst.itemClicked.connect(do_update_stat)
    lst.itemSelectionChanged.connect(do_update_stat)
    layout.addWidget(lblStat)

    # Pre-flight button
    btPrep = QPushButton("Pre-Flight")
    def do_prep():
        (uavid, uavip, uavstat) = lst.currentItemInfo()
        if uavid > 0:
            prep_aircraft(sock, uavid, my_ip, uavip)
    btPrep.clicked.connect(do_prep)
    layout.addWidget(btPrep)

    # Flight-ready button
    btToggle = QPushButton("Toggle Flight Ready")
    def do_toggle():
        (uavid, uavip, uavstat) = lst.currentItemInfo()
        if uavid > 0:
            toggle_aircraft(sock, uavid, my_ip, uavip, uavstat)
    btToggle.clicked.connect(do_toggle)
    layout.addWidget(btToggle)

    # Shutdown button
    btShutdown = QPushButton("Shut Down Payload")
    def do_shutdown():
        (uavid, uavip, uavstat) = lst.currentItemInfo()
        if uavid <= 0:
            return
        mbx = QMessageBox()
        mbx.setText("Shut down aircraft %d?" % int(uavid))
        mbx.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        mbx.setDefaultButton(QMessageBox.No)
        if mbx.exec_() == QMessageBox.Yes:
            shutdown_aircraft(sock, uavid, my_ip, uavip)
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

