#!/usr/bin/env python

from optparse import OptionParser
import sys

# Load in ACS libraries
import ap_lib.acs_messages as acs_messages
from ap_lib.acs_socket import Socket

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # Grok args
    parser = OptionParser("slave_setup.py [options]")
    parser.add_option("--device", dest="device", 
                      help="Network device to send from", default='')
    parser.add_option("--port", dest="port", type="int",
                      help="Network port to send to", default=5554)
    parser.add_option("--target", dest="target", type="int",
                      help="Target aircraft", default=0)
    parser.add_option("--enable", dest="enable",
                      help="channel to enable", default='')
    parser.add_option("--disable", dest="disable",
                      help="channel to disable", default='')
    (opts, args) = parser.parse_args()

    # Validate params
    if opts.device == '':
        print "Please use --device DEVICE"
        sys.exit(1)
    if opts.target == 0:
        print "Please use --target ID"
        sys.exit(1)
    if opts.enable == '' and opts.disable == '':
        print "Please use either --enable CHANNEL or --disable CHANNEL"
        sys.exit(1)
    elif opts.enable != '' and opts.disable != '':
        print "Cannot use --enable and --disable together"
        sys.exit(1)

    # NOTE: This is a hack to work with SITL
    my_ip = None
    bcast_ip = None
    if opts.device == 'lo':
        my_ip = '127.0.1.1'
        bcast_ip = '127.0.0.1'

    try:
        sock = Socket(0xff, opts.port, opts.device, my_ip, bcast_ip)
    except Exception:
        print "Sorry, couldn't start up the listening socket"
        sys.exit(1)

    ss = acs_messages.SlaveSetup()
    ss.msg_dst = opts.target
    ss.msg_secs = 0
    ss.msg_nsecs = 0
    ss.enable = (True if opts.enable != '' else False)
    ss.channel = (opts.enable if opts.enable != '' else opts.disable)
    sock.send(ss)

