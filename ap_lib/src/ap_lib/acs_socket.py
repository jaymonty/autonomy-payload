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
from ap_lib import acs_messages

#-----------------------------------------------------------------------
# Socket class

class Socket():
    # Constant Entity IDs
    ID_BCAST_ALL = 0xff

    # Must provide EITHER device OR (my_ip AND bcast_ip). 
    # If you provide both, ip's are used
    # Mapping IDs to IPs: mapped_ids[id] = ip_address
    def __init__(self, my_id, udp_port, device=None, 
                 my_ip=None, bcast_ip=None, mapped_ids=None):
        # Instance variables
        self.port = udp_port		# UDP port for send/recv
        self.my_id = my_id		# Local entity ID (0..255 currently)
        self.my_sub = 0			# Local entity subswarm ID
        self.id_mapping = mapped_ids	# Mapping of IDs to IPs
        self.my_ip = None		# Local entity IP address
        self.bcast_ip = None		# Broadcast IP address
        self.sock = None		# UDP socket
        
        # Attempt to look up network device addressing information
        if my_ip and bcast_ip:
            # Trust what the caller provides
            # NOTE: Use this for loopback/SITL cases
            self.my_ip = my_ip
            self.bcast_ip = bcast_ip
        else:
            try:
                self.my_ip = netifaces.ifaddresses(device)[2][0]['addr']
                self.bcast_ip = netifaces.ifaddresses(device)[2][0]['broadcast']
            except Exception:
                raise Exception("Couldn't establish IP addressing information")
        
        # Build the socket
        try:
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            if my_ip and bcast_ip:
                # For now, assume that manual IP specification implies
                #  an environment where we cannot bind to 0.0.0.0 (e.g. SITL)
                # TODO: This is sure to bite us later on, but unclear where
                self.udp_sock.bind((self.my_ip, self.port))
            else:
                self.udp_sock.bind(('', self.port))
        except Exception:
            raise Exception("Couldn't establish network socket")

    # 'msg' must be a valid Message subclass
    def send(self, msg):
        if not msg and not isinstance(msg, Message):
            raise Exception("Parameter is not a Message")
        
        # Enforce sender ID
        msg.msg_src = self.my_id
        
        # If sending to a device with a known ID->IP mapping, use it;
        #  otherwise broadcast
        dst_ip = self.bcast_ip
        if self.id_mapping and (msg.msg_dst != Socket.ID_BCAST_ALL) \
                           and (msg.msg_dst in self.id_mapping):
            dst_ip = self.id_mapping[msg.msg_dst]
        
        try:
            # Pack message into byte string
            data = msg.serialize()
            
            # Send it, return number of bytes sent (per sendto())
            return self.udp_sock.sendto(data, (dst_ip, self.port))
        except Exception as ex:
            # TODO: Raise an exception, but make sure calling code catches it
            pass

    # Return values:
    #  - <Object> - valid received message object
    #  - False - A message arrived, but one to be ignored
    #  - None - No valid message arrived
    def recv(self, buffsize=1024):
        if not buffsize:
            raise Exception("Invalid receive buffer size")
        
        try:
            data, (ip, port) = self.udp_sock.recvfrom(buffsize, socket.MSG_DONTWAIT)
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
            if ip == self.my_ip:
                return False
            
            # Parse message
            msg = acs_messages.parse(data)
            if not msg:
                return False
            
            # Is it meant for us?
            if not (msg.msg_dst == self.my_id or \
                    msg.msg_dst ==  Socket.ID_BCAST_ALL or \
                    (msg.msg_dst & 0xE0 == 0xE0 and \
                     msg.msg_dst & 0x1F == self.my_sub)):
                return False
            
            # Add source IP and port, just in case someone wants them
            msg.msg_src_ip = ip
            msg.msg_src_port = port
            
            return msg
            
        # Any other unhandled conditions are printed for our awareness
        except Exception as ex:
            print(ex.args[0])

