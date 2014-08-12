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
import acs_messages

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

    # 'message' must be a valid Message subclass
    def send(self, message):
        if not message and not isinstance(message, Message):
            raise Exception("Parameter is not a Message")
        
        # Enforce sender ID
        message.msg_src = self.my_id
        
        # If sending to a device with a known ID->IP mapping, use it;
        #  otherwise broadcast
        dst_ip = self.bcast_ip
        if self.id_mapping and (message.msg_dst != Socket.ID_BCAST_ALL) \
                           and (message.msg_dst in self.id_mapping):
            dst_ip = self.id_mapping[message.msg_dst]
        
        try:
             # Pack message into byte string
            msg = struct.pack(message.hdr_fmt, *message.build_hdr_tuple())
            if message.msg_size:
                msg = msg + struct.pack(message.msg_fmt, *message.build_tuple())
            
            # Send it, return number of bytes sent (per sendto())
            return self.udp_sock.sendto(msg, (dst_ip, self.port))
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
            msg, (ip, port) = self.udp_sock.recvfrom(buffsize, socket.MSG_DONTWAIT)
            # Mostly likely due to no packets being available
            if not msg:
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
            
            # Parse header
            # TODO: This should be done by the Message sub-class,
            #  but we won't know which subclass to generate until we
            #  get the message type :(
            msg_type, _unused, msg_src, msg_dst, msg_secs, msg_nsecs = \
                struct.unpack_from(acs_messages.Message.hdr_fmt, msg, 0)
            
            # Is it meant for us?
            if msg_dst not in [self.my_id, Socket.ID_BCAST_ALL]:
                return False
            
            # Is it a valid type?
            message = acs_messages.generate_message_object(msg_type)
            if message is None:
                return False
            
            # Populate message object with headers
            message.msg_type = msg_type
            message.msg_src = msg_src
            message.msg_dst = msg_dst
            message.msg_secs = msg_secs
            message.msg_nsecs = msg_nsecs

            # Add source IP and port, just in case someone wants them
            message.msg_src_ip = ip
            message.msg_src_port = port
            
            # If message type has payload fields, parse and populate them
            if message.msg_size and (len(msg) == (message.hdr_size + message.msg_size)):
                fields = struct.unpack_from(message.msg_fmt, msg, message.hdr_size)
                message.parse_tuple(fields)
            
            return message
            
        # Any other unhandled conditions, pass them along
        except Exception as ex:
            print(ex.args[0])
            pass

