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

def create_sock(bind_ip, broadcast_ip, port):
    global socks
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        socks[s.fileno()] = (s, broadcast_ip)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind((bind_ip, port))
        print "Opened listening socket %s:%u" % (bind_ip, port)
    except Exception as ex:
        print "Error opening socket %s:%u: %s" % (bind_ip, port, ex.args[0])
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
    parser.add_argument('-p', '--payload-ip', dest='payload_ip', default='127.0.0.1',
                        help='IP that the payloads listen on')
    parser.add_argument('-r', '--repeater-ip', dest='repeater_ip', default='127.0.1.1',
                        help='IP that the repeater listens on')
    parser.add_argument('-I', '--broadcast-ip', dest='broadcast_ip', default=None,
                        help='Real-network broadcast IP to use')
    parser.add_argument('-P', '--broadcast-port', dest='broadcast_port', default=None,
                        type=int, help='Real-network broadcast port to use')
    parser.add_argument('-B', '--buffer-size', dest='buffer_size', default=2048,
                        type=int, help='Receive buffer size in bytes')
    parser.add_argument('num_ports', type=int, help='Number of ports to open')
    args = parser.parse_args()

    # Attempt to stand up all sockets
    socks = {}
    for port in range(args.base_port, args.base_port+args.num_ports):
        create_sock(args.repeater_ip, args.payload_ip, port)
    for port in args.extra_ports:
        create_sock(args.repeater_ip, args.payload_ip, port)
    if args.broadcast_ip is not None and args.broadcast_port is not None:
        create_sock('0.0.0.0', args.broadcast_ip, args.broadcast_port)
    elif args.broadcast_ip is not None or args.broadcast_port is not None:
        print "Warning: you must specify an IP *and* port for real-network broadcast."

    # Loop, waiting for a socket to have data to read, and repeat out all other ports
    while True:
        # Use select to wait for any socket to be ready with a message
        rready = []
        try:
            rready, wready, xready = select.select(socks.keys(), [], [])
        except KeyboardInterrupt:
            break
        except Exception as ex:
            print "Select error: " + ex.args[0]
            continue

        # Iterate through all ready messages
        for r in rready:
            # Receive the message
            try:
                # NOTE: may need to increase buffer size eventually
                data, (recv_ip, recv_port) = socks[r][0].recvfrom(args.buffer_size)
            except Exception as ex:
                print "Receive error (%u): %s" % (port, ex.args[0])
                continue

            # Iterate through all sockets and relay message
            for fd in socks:
                try:
                    # Look up all IP and port info
                    sock, bcast_ip = socks[fd]
                    send_port = sock.getsockname()[1]

                    # Don't send to self
                    if fd != r:
                        sock.sendto(data, (bcast_ip, send_port))
                except Exception as ex:
                    print "Send error (%u -> %u): %s" % \
                          (recv_port, send_port, ex.args[0])

    shut_down_socks()
    sys.exit(0)

