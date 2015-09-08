#!/usr/bin/env python3

from optparse import OptionParser
import os
import sys
import time

# Load in ACS libraries
import ap_lib.acs_messages as messages
from ap_lib.acs_socket import Socket
import ap_lib.ap_enumerations as enums

class VehicleStats(object):
    def __init__(self, id, last_time):
        self.id = id
        self.last_time = last_time
        self.status = messages.FlightStatus()
        self.intent = messages.VehicleIntent()

        # Initialize to avoid startup erros
        self.status.alt_rel = 0
        self.intent.alt = 0
        self.status.batt_rem = 0

    def update(self, message):
        if isinstance(message, messages.FlightStatus):
            self.status = message
            self.status.alt_rel /= 1000.0
        elif isinstance(message, messages.VehicleIntent):
            self.intent = message
        self.last_time = time.time()

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
        print("Sorry, couldn't start up the listening socket")
        sys.exit(1)

    swarm_stats = dict()
    swarm = []
    last_time = 0

    while True:
        message = sock.recv()

        # Parse network message
        if message != None and message != False and message.msg_src != 173 and message.msg_src != 183:
            id = message.msg_src
            if id not in swarm_stats:
                swarm_stats[id] = VehicleStats(id, time.time())
            swarm_stats[id].update(message)
        else:
            time.sleep(0.1)

        del swarm[:]
        time_now = time.time()

        # Only add active planes
        [swarm.append(swarm_stats[vehicle]) for vehicle in swarm_stats
            if time_now - swarm_stats[vehicle].last_time < 10]

        # Update at 10 Hz
        if time_now - .25 > last_time:
            last_time = time_now
            os.system('clear')

            swarm.sort(key=lambda vehicle: vehicle.id)

            # Show summary counts
            print ("TOTALS")
            print ("Powered: \t %2u" % len(swarm))
            print ("Ready: \t\t %2u" % len([v for v in swarm if v.status.ready]))
            print ("Aloft: \t\t %2u" % len([v for v in swarm if v.status.alt_rel > 25.]))

            print ("\nIN-AIR MODES")
            print ("RTL: \t\t %2d" % len([v for v in swarm if v.status.mode is enums.RTL and v.status.alt_rel > 25.]))
            print ("GUIDED: \t %2d" % len([v for v in swarm if v.status.mode is enums.GUIDED and v.status.alt_rel > 25.]))
            print ("MANUAL: \t %2d" % len([v for v in swarm if v.status.mode is enums.MANUAL and v.status.alt_rel > 25.]))
            print ("disarmed: \t %2d" % len([v for v in swarm if not v.status.armed and v.status.alt_rel > 25.]))

            # Show landing planes
            swarm.sort(key=lambda vehicle: vehicle.status.alt_rel)
            def isLanding(uav, l, h):
                if uav is None: return False
                if uav.status is None: return False
                if uav.status.mis_cur is None: return False
                if uav.status.alt_rel is None: return False
                return bool(uav.status.alt_rel > 5.0 and l <= uav.status.mis_cur <= h)
            if len([v for v in swarm if isLanding(v, 13, 17)]) > 0:
                print("\nLAND A")
            [print("UAV %s, altitude: %.3f" % (uav.id, uav.status.alt_rel))
                for uav in swarm if isLanding(uav, 13, 17)]

            if len([v for v in swarm if isLanding(v, 19, 23)]) > 0:
                print("\nLAND B")
            [print("Aircraft %s, altitude: %.3f" %(uav.id, uav.status.alt_rel))
                for uav in swarm if isLanding(uav, 19, 23)]

            print("")
            # Show battery level info for aircraft aloft
            swarm.sort(key=lambda vehicle: vehicle.status.batt_rem)
            for ndx in range(min(5, len(swarm))):
                if 30 <= swarm[ndx].status.batt_rem < 50:
                    print ("\033[1;33mBATT ALERT: %d for UAV %d\033[0m" % \
                        (swarm[ndx].status.batt_rem, swarm[ndx].id))
                elif 0 < swarm[ndx].status.batt_rem < 30:
                    print ("\033[1;31mBATT ALERT: %d for UAV %d\033[0m" % \
                        (swarm[ndx].status.batt_rem, swarm[ndx].id))

            print("")

            # Show any colliding slots
            #     print ("Intent Conflicts")
            # swarm.sort(key=lambda vehicle: vehicle.intent.alt)
            # for ndx in range(len(swarm)-1):
            #     if swarm[ndx].intent.alt == swarm[ndx+1].intent.alt and \
            #        swarm[ndx].intent.alt != 0 and swarm[ndx].status.alt_rel > 90 \
            #        and swarm[ndx].status.alt_rel > 90:
            #         print ("Planes %s and %s" % (swarm[ndx].id, swarm[ndx+1].id))

        sys.stdout.flush()
