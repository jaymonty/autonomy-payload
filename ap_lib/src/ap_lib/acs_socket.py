#!/usr/bin/env python

#-----------------------------------------------------------------------
# ACS Comms Library
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
import netifaces
import socket
import struct

# Message definitions
import ap_lib.acs_messages as messages

#-----------------------------------------------------------------------
# Socket class

class Socket():
    # Constant Entity IDs
    ID_BCAST_ALL = 0xff

    # Must provide EITHER device OR (my_ip AND bcast_ip). 
    # If you provide both, ip's are used
    # Mapping IDs to IPs: mapped_ids[id] = ip_address
    def __init__(self, my_id, udp_port, device=None, 
                 my_ip=None, bcast_ip=None, mapped_ids=None, send_only=False):
        # Instance variables
        self._port = udp_port		# UDP port for send/recv
        self._id = my_id		# Local entity ID (0..255 currently)
        self._subswarm = 0		# Local entity subswarm ID
        self._idmap = mapped_ids	# Mapping of IDs to IPs
        self._ip = my_ip		# Local entity IP address
        self._bcast = bcast_ip		# Broadcast IP address
        self._sock = None		# UDP socket
        self._sendonly = send_only	# Don't bind a port
        
        # If user did not specify both addresses,
        # Attempt to look up network device addressing information
        if not my_ip or not bcast_ip:
            try:
                self._ip = netifaces.ifaddresses(device)[2][0]['addr']
                self._bcast = netifaces.ifaddresses(device)[2][0]['broadcast']
            except Exception:
                raise Exception("Couldn't establish IP addressing information")
        
        # Build the socket
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            if send_only:
                # Allow a socket that can send but not receive
                pass
            elif my_ip and bcast_ip:
                # For now, assume that manual IP specification implies
                #  an environment where we cannot bind to 0.0.0.0 (e.g. SITL)
                # TODO: This is sure to bite us later on, but unclear where
                self._sock.bind((self._ip, self._port))
            else:
                self._sock.bind(('', self._port))
        except Exception as ex:
            raise Exception("Socket init error: %s" % ex.args[0])

    # Get and set subswarm ID in Pythonic property style
    # NOTE: Apparently must reference internally by property name,
    #  *not* by internal variable name. Not clear why.
    @property
    def subswarm(self):
        return self._subswarm
    @subswarm.setter
    def subswarm(self, value):
        self._subswarm = value

    # 'msg' must be a valid Message subclass
    def send(self, msg):
        if not msg and not isinstance(msg, Message):
            raise Exception("Parameter is not a Message")
        
        # Enforce sender ID and subswarm ID
        msg.msg_src = self._id
        msg.msg_sub = self.subswarm
        
        # If sending to a device with a known ID->IP mapping, use it;
        #  otherwise broadcast
        dst_ip = self._bcast
        if self._idmap and (msg.msg_dst != Socket.ID_BCAST_ALL) \
                       and (msg.msg_dst in self._idmap):
            dst_ip = self._idmap[msg.msg_dst]
        
        try:
            # Pack message into byte string
            data = msg.serialize()
            
            # Send it, return number of bytes sent (per sendto())
            return self._sock.sendto(data, (dst_ip, self._port))

        # Print any exception for user's awareness
        except Exception as ex:
            print ex.args[0]
            return False

    # Return values:
    #  - <Object> - valid received message object
    #  - False - A message arrived, but one to be ignored
    #  - None - No valid message arrived
    def recv(self, buffsize=1024):
        if self._sendonly:
            raise Exception("Attempted to receive on send-only socket")

        if not buffsize:
            raise Exception("Invalid receive buffer size")
        
        try:
            data, (ip, port) = self._sock.recvfrom(buffsize, socket.MSG_DONTWAIT)
            # Mostly likely due to no packets being available
            if not data:
                return None
        except Exception as ex:
            # Mostly likely due to no packets being available
            return None
        
        # If anything goes wrong below, return False so caller knows
        #  there may be more packets to receive
        try:
            # Ignore packets we sent (we see our own broadcasts)
            if ip == self._ip:
                return False
            
            # Parse message
            msg = messages.Message.parse(data)
            
            # Is it meant for us?
            if not (msg.msg_dst == self._id or \
                    msg.msg_dst == Socket.ID_BCAST_ALL or \
                    (msg.msg_dst & messages.SUBSWARM_MASK == messages.SUBSWARM_MASK and \
                     msg.msg_dst & messages.SUBSWARM_BITS == self.subswarm)):
                return False
            
            # Add source IP and port, just in case someone wants them
            msg.msg_src_ip = ip
            msg.msg_src_port = port
            
            return msg
            
        # Any other unhandled conditions are printed for our awareness
        except Exception as ex:
            print("sock recv: %s" % ex.args[0])
            return False

