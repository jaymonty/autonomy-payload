#!/usr/bin/env python

from optparse import OptionParser
import os
import sys
import time

# Load in ACS libraries
import ap_lib.acs_messages as messages
from ap_lib.acs_socket import Socket

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("net_parser.py [options]")
    parser.add_option("-d", "--device", dest="device", 
                      help="Network device to listen on", default='')
    parser.add_option("-p", "--port", dest="port", type="int",
                      help="Network port to listen on", default=5554)
    parser.add_option("--repeat", dest="repeat",
                      action="store_true", default=False)
    parser.add_option("--lo-reverse", dest="lo_reverse",
                      action="store_true", default=False,
                      help="If using lo, reverse the addresses")
    (opts, args) = parser.parse_args()
    
    # NOTE: This is a hack to work with SITL
    my_ip = None
    bcast_ip = None
    if opts.device == 'lo':
        my_ip = '127.0.1.1'
        bcast_ip = '127.0.0.1'
        if opts.lo_reverse:
            (my_ip, bcast_ip) = (bcast_ip, my_ip)
    
    try:
        sock = Socket(0xff, opts.port, opts.device, my_ip, bcast_ip, promisc=True)
    except Exception:
        print "Sorry, couldn't start up the listening socket"
        sys.exit(1)
    
    while True:
        message = sock.recv()
        if message == None:  # No packets to receive, wait a bit
            time.sleep(0.1)
            continue
        elif message == False:  # Packet received, but ignored
            print "<Ignored packet>"
            continue
        
        print "ID: %u > %u TYPE: %02X SEQ/ACK: %03u/%03u FLAGS: %u%u%u TIME: %u.%09u" % \
            (message.msg_src, message.msg_dst, message.msg_type,
             message.msg_seq, message.msg_ack,
             message.msg_fl_rel, message.msg_fl_syn, 0,
             message.msg_secs, message.msg_nsecs)
        
        if isinstance(message, messages.Pose):
            print "\tLa: %f Lo: %f Al: %f Qx: %f Qy: %f Qz: %f Qw: %f\n\tVLx: %f VLy: %f VLz: %f VAx: %f VAy: %f VAz: %f" % \
                (message.lat, message.lon, message.alt, 
                 message.q_x, message.q_y, message.q_z, message.q_w,
                 message.vlx, message.vly, message.vlz,
                 message.vax, message.vay, message.vaz)
        elif isinstance(message, messages.FlightStatus):
            ready_string = ''
            for b in message.ctl_ready[1:]:
                ready_string += "%u" % int(b)
            print "\tNm: %s Su: %u Mo: %u Wp: %u Fl: %u%u%u%u%u%u%u%u%u Br: %d Bv: %d As: %f Ar: %d Cm: %u Cr: %s" % \
                (message.name, message.msg_sub, message.mode,
                 int(message.mis_cur), int(message.armed), int(message.ok_ahrs),
                 int(message.ok_as), int(message.ok_gps), int(message.ok_ins),
                 int(message.ok_mag), int(message.ok_pwr), int(message.ready),
                 int(message.swarming),
                 message.batt_rem, message.batt_vcc,
                 message.airspeed, message.alt_rel,
                 message.ctl_mode, ready_string)
        else:
            print "\t<I don't know how to decode this type>"

        sys.stdout.flush()
        
        # Debug code: repeat message (will take on false source ID)
        if opts.repeat:
            sock.send(message)

