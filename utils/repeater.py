#!/usr/bin/env python

# UDP packet repeater
# Written by Mike Clement, October 2014
# Revised by Mike Clement, May 2015

from argparse import ArgumentParser
from netifaces import ifaddresses
from select import poll, POLLIN
import socket
import sys

#-----------------------------------------------------------------------
# UDPRepeater class

class UDPRepeater(object):

    ''' Socket classes '''

    # Internal class for managing a socket
    class _Socket(object):

        # Struct-like class for holding an address/port pair
        class _AddressPort(object):

            def __init__(self, ip, port):
                self.ip = ip            # Remote IP
                self.port = port        # Remote Port

            def __eq__(self, other):
                return self.ip == other.ip and \
                       self.port == other.port

            def __str__(self):
                return "%s:%u" % (self.ip, self.port)

        def __init__(self, bind_ip, bind_port, buf_size):
            # Build the socket (always set broadcast flag)
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self._sock.bind((bind_ip, bind_port))

            # Store info about the socket
            self._local = self._AddressPort(bind_ip, bind_port)
            self._buf_size = buf_size

        def __str__(self):
            return str(self._local)

        def _recv(self):
            return self._sock.recvfrom(self._buf_size)

        def _send(self, data, dest):
            return self._sock.sendto(data, dest)

        # Return the file descriptor of the socket
        def fd(self):
            return self._sock.fileno()

        def close(self):
            self._sock.close()

        # Handle a packet, including resending on this socket as appropriate
        # If there is an error you want to report, raise an exception
        # If the packet should be repeated on other sockets, return its data
        # If the packet should be discarded, return None
        def handlePacket(self):
            raise Exception("You haven't implemented this function!")

        # Send a packet everywhere this socket knows how to send to
        # If there is an error you want to report, raise an exception
        def sendPacket(self, data):
            raise Exception("You haven't implemented this function!")

    # Specialization of _Socket for unicasting with a single remote endpoint
    class _EndpointSocket(_Socket):

        def __init__(self, loc_ip, loc_port, rem_ip, rem_port, buf_size):
            UDPRepeater._Socket.__init__(self, loc_ip, loc_port, buf_size)
            self._remote = self._AddressPort(rem_ip, rem_port)

        def handlePacket(self):
            data, (src_ip, src_port) = self._recv()
            return data

        def sendPacket(self, data):
            self._send(data, (self._remote.ip, self._remote.port))

    # Specialization of _Socket for broadcasting on a single device
    class _DeviceSocket(_Socket):

        def __init__(self, device, loc_port, rem_port, buf_size):
            # Get address info for the device
            local_ip = ifaddresses(device)[2][0]['addr']
            bcast_ip = ifaddresses(device)[2][0]['broadcast']

            UDPRepeater._Socket.__init__(self, bcast_ip, loc_port, buf_size)
            self._remote = self._AddressPort(bcast_ip, rem_port)

            # Store local address for filtering later
            self._local_ip = local_ip

        def handlePacket(self):
            data, (src_ip, src_port) = self._recv()

            # Discard reflected broadcast packets
            if src_ip == self._local_ip and \
               src_port == self._local.port:
                return None

            return data

        def sendPacket(self, data):
            self._send(data, (self._remote.ip, self._remote.port))

    ''' UDPRepeater functions '''

    def __init__(self, recv_buf_sz=2048):
        # Store _Socket objects indexed by their file descriptor
        self._sock_list = {}

        # Polling object
        self._poll = poll()

        # Receive buffer size
        self._recv_buf_sz = recv_buf_sz

    # Register a socket for polling and add it to the list of fd's
    def _add(self, sock):
        self._poll.register(sock.fd(), POLLIN)
        self._sock_list[sock.fd()] = sock

    # Add a socket by endpoint information
    def addEndpoint(self, loc_ip, rem_ip, port, rem_port=None):
        # Default to symmetric port numbers
        if rem_port is None: rem_port = port
        sock = self._EndpointSocket(loc_ip, port, rem_ip, rem_port,
                                    self._recv_buf_sz)
        self._add(sock)

    # Add a socket for a broadcast-capable device
    def addDevice(self, device, port, rem_port=None):
        # Default to symmetric port numbers
        if rem_port is None: rem_port = port
        sock = self._DeviceSocket(device, port, rem_port,
                                  self._recv_buf_sz)
        self._add(sock)

    # Close all sockets and clear socket list
    # NOTE: Can reuse object, but must call add*() over again
    def closeAll(self):
        for s in self._sock_list:
            try:
                self.poll.unregister(s)
                self._sock_list[s].close()
            except:
                pass
        del self._sock_list
        self._sock_list = {}

    # Repeat packets across interfaces until program is terminated
    def runLoop(self):
        if not len(self._sock_list):
            raise Exception("Cannot loop without adding any devices or ports")

        while True:
            # Blocking wait for one or more sockets to have data to receive
            try:
                # Blocking wait for socket(s) to have data
                ready = self._poll.poll()
            except KeyboardInterrupt:
                # Stop looping if user presses Ctrl+c
                # NOTE: a signal handler is probably cleaner here
                break
            except Exception as ex:
                # Treat other exceptions as aberrant and not fatal
                print "Polling error: " + str(ex)
                continue

            # Iterate through ready sockets, handling one packet from each
            for r_fd, r_ev in ready:
                # Only process read-available events
                if r_ev != POLLIN: continue

                try:
                    data = self._sock_list[r_fd].handlePacket()
                except Exception as ex:
                    print "Receive error (%s): %s" % \
                          (str(self._sock_list[r_fd]), str(ex))
                    continue

                # If handlePacket() returned None, move on
                if data is None:
                    #print "Ignored from " + str(self._sock_list[r_fd])
                    continue

                #print "Got from " + str(self._sock_list[r_fd])

                # Iterate through all *other* sockets and repeat message
                for fd, s in self._sock_list.iteritems():
                    if fd == r_fd: continue
                    try:
                        #print "Send to " + str(self._sock_list[fd])
                        s.sendPacket(data)
                    except Exception as ex:
                        print "Send error (%s): %s" % \
                              (str(self._sock_list[r_fd]), str(ex))

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
    parser.add_argument('-D', '--broadcast-device', dest='devices', default=[],
                        action='append', help='Real-network broadcast device to use')
    parser.add_argument('-P', '--broadcast-port', dest='bcast_port', default=5554,
                        type=int, help='Real-network broadcast port to use')
    parser.add_argument('-B', '--buffer-size', dest='buffer_size', default=2048,
                        type=int, help='Receive buffer size in bytes')
    parser.add_argument('num_ports', type=int, help='Number of ports to open')
    args = parser.parse_args()

    # Stand up repeater object
    repeater = UDPRepeater(args.buffer_size)

    # Add broadcast devices
    for device in args.devices:
        d, p = device, args.bcast_port
        if ':' in d:
            d, p = device.split(':')
        try:
            repeater.addDevice(d, int(p))
            print "Added device '%s:%u'" % (d, int(p))
        except Exception as ex:
            print "Error adding device '%s:%u': %s" % (d, p, str(ex))
            repeater.closeAll()
            sys.exit(1)

    # Add IP-IP port pairs
    for port in range(args.base_port, args.base_port+args.num_ports) + \
                args.extra_ports:
        try:
            repeater.addEndpoint(args.repeater_ip, args.payload_ip, port)
            print "Added endpoint '%s:%u -- %s:%u'" % \
                  (args.repeater_ip, port, args.payload_ip, port)
        except Exception as ex:
            print "Error adding endpoint '%s:%u -- %s:%u': %s" % \
                  (args.repeater_ip, port, args.payload_ip, port, str(ex))
            repeater.closeAll()
            sys.exit(1)

    # Loop until user exits
    print "\nStarting repeater ...\n"
    try:
        repeater.runLoop()
    except Exception as ex:
        print "Fatal loop error: " + str(ex)

    # Shut everything down
    print "Shutting down repeater ..."
    repeater.closeAll()
    sys.exit(0)

