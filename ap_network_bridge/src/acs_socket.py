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
import ap_messages

#-----------------------------------------------------------------------
# Ugly global variables

# Entity IDs
# 0x00 - 0xdf are aerial entities
ID_SELF = None
# 0xe0 - 0xef are ground entities
ID_GCS = 0xe0
ID_ARBITER = 0xef
# 0xf0 - 0xff are bcast groups
ID_BCAST_ALL = 0xff

# UDP socket and info
udp_sock = None
udp_sock_local_ip = None
udp_sock_bcast_ip = None
udp_sock_gcs_ip = None
udp_sock_arbiter_ip = None
udp_sock_port = None

#-----------------------------------------------------------------------
# Messaging functions

def init(device, port, local_id=None, gcs_ip=None, arbiter_ip=None):
    global udp_sock, udp_sock_local_ip, udp_sock_port
    global udp_sock_gcs_ip, udp_sock_arbiter_ip, udp_sock_bcast_ip
    global id_self

    udp_sock_port = port
    udp_sock_gcs_ip = gcs_ip
    udp_sock_arbiter_ip = arbiter_ip
    ID_SELF = local_id

    # Attempt to look up network device addressing information
    if device == 'lo':
        # Assumes 'lo' is only used for single-UAV testing with SITL,
        # broadcast solution doesn't really work with multiple UAS
        udp_sock_local_ip = '127.0.0.1'
        udp_sock_bcast_ip = '127.0.1.1'
    else:
        try:
            udp_sock_local_ip = netifaces.ifaddresses(device)[2][0]['addr']
            udp_sock_bcast_ip = netifaces.ifaddresses(device)[2][0]['broadcast']
        except Exception:
            log_warn("Could not get address info for device %s!"%device)
            return False
    
    # If these weren't set above, default to broadcast
    if not udp_sock_gcs_ip:
        udp_sock_gcs_ip = udp_sock_bcast_ip
    if not udp_sock_arbiter_ip:
        udp_sock_arbiter_ip = udp_sock_bcast_ip
    
    # Build the socket
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    udp_sock.bind((udp_sock_local_ip, udp_sock_port))
    return True

def send(message):
    global udp_sock, udp_sock_port

    if not message:
        return False
    
    # Enforce sender ID
    message.msg_src = ID_SELF
    
    try:
        # Pack message into byte string
        msg = struct.pack(message.hdr_fmt, *message.build_hdr_tuple())
        if message.msg_size:
            msg = msg + struct.pack(message.msg_fmt, *message.build_tuple())
        
        # Send it
        return udp_sock.sendto(msg, (ip, udp_sock_port))
    except Exception:
        return False

# Return values:
#  - <Object> - valid received message object
#  - False - A message arrived, but one to be ignored
#  - None - No valid message arrived
def recv():
    global udp_sock, udp_sock_local_ip
    global id_self, id_bcast

    try:
        # Attempt to receive a packet
        msg, (ip, port) = udp_sock.recvfrom(1024, socket.MSG_DONTWAIT)
        if not msg:
            return None
        
        # Ignore packets we sent
        if ip == udp_sock_local_ip:
            return False
        
        # Parse header
        msg_type, _unused, msg_src, msg_dst, msg_secs, msg_nsecs = \
            struct.unpack_from(HEADER_FORMAT, msg, 0)
        
        # Is it meant for us?
        if msg_id not in [id_self, id_bcast]:
            return False
        
        # Is it a valid type?
        message = generate_message_object(msg_type)
        if message is None:
            return False
        
        # Populate message object with headers
        message.msg_type = msg_type
        message.msg_src = msg_src
        message.msg_dst = msg_dst
        message.msg_secs = msg_secs
        message.msg_nsecs = msg_nsecs
        
        # If message type has payload fields, parse and populate them
        if message.msg_size and (len(msg) == (message.hdr_size + pload_size)):
            fields = struct.unpack_from(message.msg_fmt, msg, message.hdr_size)
            message.parse_tuple(fields)
        
        return message
        
    except Exception as ex:
        return None

