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
# Constants

# Constant Entity IDs
ID_BCAST_ALL = 0xff

#-----------------------------------------------------------------------
# Ugly global variables

# Entity identifiers
id_self = None
id_mapping = None

# UDP socket and info
udp_sock = None
udp_sock_local_ip = None
udp_sock_bcast_ip = None
udp_sock_port = None

#-----------------------------------------------------------------------
# Messaging functions

# Must provide EITHER device OR (local_ip, bcast_ip)
# mapped_ids[id] = ip_address
def init(device, port, my_id=None, mapped_ids=None, local_ip=None, bcast_ip=None):
    global udp_sock, udp_sock_local_ip, udp_sock_bcast_ip, udp_sock_port
    global id_self, id_mapping
    
    # Set socket parameters
    udp_sock_port = port
    id_self = my_id
    id_mapping = mapped_ids
    
    # Attempt to look up network device addressing information
    if local_ip and bcast_ip:
        # Trust what the caller provides
        udp_sock_local_ip = local_ip
        udp_sock_bcast_ip = bcast_ip
    elif device == 'lo':
        # Assumes 'lo' is only used for single-UAV testing with SITL,
        # broadcast solution doesn't really work with multiple UAS
        udp_sock_local_ip = '127.0.0.1'
        udp_sock_bcast_ip = '127.0.1.1'
    else:
        try:
            udp_sock_local_ip = netifaces.ifaddresses(device)[2][0]['addr']
            udp_sock_bcast_ip = netifaces.ifaddresses(device)[2][0]['broadcast']
        except Exception:
            return False
    
    # Build the socket
    try:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        udp_sock.bind((udp_sock_local_ip, udp_sock_port))
        return True
    except Exception:
        return False

def send(message):
    global udp_sock, udp_sock_port, udp_sock_bcast_ip
    global id_self, id_mapping, ID_BCAST_ALL

    if not (udp_sock and udp_sock_port and message):
        return False
    
    # Enforce sender ID
    message.msg_src = id_self
    
    # If sending to a device with a known ID->IP mapping, use it;
    #  otherwise broadcast
    dst_ip = udp_sock_bcast_ip
    if id_mapping and (message.msg_dst != ID_BCAST_ALL) \
                  and (message.msg_dst in id_mapping):
        dst_ip = id_mapping[message.msg_dst]
    
    try:
        # Pack message into byte string
        msg = struct.pack(message.hdr_fmt, *message.build_hdr_tuple())
        if message.msg_size:
            msg = msg + struct.pack(message.msg_fmt, *message.build_tuple())
        
        # Send it
        return udp_sock.sendto(msg, (dst_ip, udp_sock_port))
    except Exception:
        return False

# Return values:
#  - <Object> - valid received message object
#  - False - A message arrived, but one to be ignored
#  - None - No valid message arrived
def recv(buffsize=1024):
    global udp_sock, udp_sock_local_ip
    global id_self, ID_BCAST_ALL

    if not (udp_sock and buffsize):
        return None
    
    # Attempt to receive a packet, return None if any issue so
    #  caller knows to wait a bit, or error
    try:
        msg, (ip, port) = udp_sock.recvfrom(buffsize, socket.MSG_DONTWAIT)
        if not msg:
            return None
    except Exception:
        # Mostly likely due to no packets being available
        return None
    
    # If anything goes wrong below, return False so caller knows
    #  there may be more packets to receive
    try:
        # Ignore packets we sent
        if ip == udp_sock_local_ip:
            return False
        
        # Parse header
        # TODO: This should be done by the Message sub-class,
        #  but we won't know what type to generate until we
        #  get the message type :(
        msg_type, _unused, msg_src, msg_dst, msg_secs, msg_nsecs = \
            struct.unpack_from(acs_messages.Message.hdr_fmt, msg, 0)
        
        # Is it meant for us?
        if msg_dst not in [id_self, ID_BCAST_ALL]:
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
        
        # If message type has payload fields, parse and populate them
        if message.msg_size and (len(msg) == (message.hdr_size + message.msg_size)):
            fields = struct.unpack_from(message.msg_fmt, msg, message.hdr_size)
            message.parse_tuple(fields)
        
        return message
        
    except Exception as ex:
        print ex.args
        return False

