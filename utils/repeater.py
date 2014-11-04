#!/usr/bin/env python

# UDP packet repeater
# Written by Mike Clement, October 2014
#
# Can be used to run multiple *local* payload instances without
# having to start up containers.

from argparse import ArgumentParser
import sys
import socket
import select

socks = {}

def create_sock(ip, port):
    global socks
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        socks[s.fileno()] = s
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind((ip, port))
        print "Opened listening socket at port %u" % port
    except Exception as ex:
        print "Error opening socket at port %u: %s" % (port, ex.args[0])
        shut_down_socks()
        sys.exit(-1)

def shut_down_socks():
    global socks
    print "Shutting down sockets..."
    for s in socks.values():
        try:
            s.close()
        except:
            pass



#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    # Grok args
    parser = ArgumentParser("python %s" % sys.argv[0])
    parser.add_argument('-b', '--base-port', dest='base_port', default=5555,
                        type=int, help='Lowest port number to use')
    parser.add_argument('-e', '--extra-port', dest='extra_ports', default=[],
                        action='append', type=int, help='Open this extra port')
    parser.add_argument('-r', '--repeater-ip', dest='repeater_ip', default='127.0.1.1',
                        help='IP that the repeater listens on')
    parser.add_argument('-p', '--payload-ip', dest='payload_ip', default='127.0.0.1',
                        help='IP that the payloads listen on')
    parser.add_argument('num_ports', type=int, help='Number of ports to open')
    args = parser.parse_args()

    # Attempt to stand up all sockets
    socks = {}
    for port in range(args.base_port, args.base_port+args.num_ports):
        create_sock(args.repeater_ip, port)
    for port in args.extra_ports:
        create_sock(args.repeater_ip, port)

    # Loop, waiting for a socket to have data to read, and repeat out all other ports
    while True:
        rready = []
        try:
            rready, wready, xready = select.select(socks.keys(), [], [])
        except KeyboardInterrupt:
            break
        except Exception as ex:
            print "Select error: " + ex.args[0]
            continue

        for r in rready:
            try:
                data, (ip, port) = socks[r].recvfrom(2048, socket.MSG_DONTWAIT)
            except Exception as ex:
                print "Receive error (%u): %s" % (port, ex.args[0])
                continue

            for s in socks.values():
                try:
                    p = s.getsockname()[1]
                    if p != port:
                        s.sendto(data, (args.payload_ip, p))
                except Exception as ex:
                    print "Send error (%u -> %u): %s" % (port, p, ex.args[0])
                    continue

    shut_down_socks()
    sys.exit(0)

