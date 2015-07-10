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
    parser = OptionParser("takeoff_moniter.py [options]")
    parser.add_option("-d", "--device", dest="device",
                      help="Network device to listen on", default='')
    parser.add_option("-p", "--port", dest="port", type="int",
                      help="Network port to listen on", default=5554)
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


    uav_list = {}
    last_sec = 0

    while True:
        message = sock.recv()
        if message == None:  # No packets to receive, wait a bit
            time.sleep(0.1)
            continue
        elif message == False:  # Packet received, but ignored
            continue

        uav = str(message.msg_src)

        if uav not in uav_list:
            uav_list[uav] = [0, 0, time.time()]

        # Update last time heard
        uav_list[uav][2] = time.time()
        if isinstance(message, messages.FlightStatus):
            uav_list[uav][0] = message.alt_rel/1000.
        elif isinstance(message, messages.AltitudeSlot):
            uav_list[uav][1] = message.altitude_slot


        t = time.time()
        if t - .5 > last_sec:
            last_sec = int(t)
            os.system('clear')
            # If a message is old then delete it else print it out
            for uav in uav_list.keys():
                if uav_list[uav][2] + 5 < time.time():
                    del uav_list[uav]
                else:
                    print ("UAV %s \t Alt: %.3f \t Slot %d" % \
                        (uav, uav_list[uav][0], uav_list[uav][1]))

            for idx in uav_list:
                for ndx in uav_list:
                    if idx != ndx and uav_list[idx][1] == uav_list[ndx][1] \
                        and uav_list[idx][1] != 0:
                        print "\033[1;31mERROR: \033[96m"+str(idx)+"\033[94m and \033[96m"+str(ndx)+ \
                            "\033[94m have conflicting slots at \033[1;31m"+str(uav_list[idx][1])+"\033[0m"

        sys.stdout.flush()
