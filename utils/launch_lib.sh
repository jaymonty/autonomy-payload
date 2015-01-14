#!/bin/bash

# Configuration and functions for launching a payload



### Configuration Variables ###

# This applies for container mode and for creating a SITL bridge.
# Set up the basic networking parameters for our multi-SITL environment.
# This is set to mimic our flight configuration as closely as possible.
# All aircraft (and ground stations) are in 192.168.2.0/24

NET_IP_PREFIX="192.168.2"
NET_MASK="255.255.255.0"
NET_BCAST="${NET_IP_PREFIX}.255"

# Container Mode: Network namespaces need unique names, and new network
# interfaces need namespace-unique names. These are the prefixes; a number
# will be appended to each where appropriate.

NET_DEV_PREFIX="sitl"
NET_NS_PREFIX="sitl"

# SITL Bridge: Parameters for a soft network bridge that can be a virtual
# network interface capable of broadcast, and can be a bridge between
# containers when using container mode.

BRIDGE_DEV="sitl_bridge"
BRIDGE_IP="${NET_IP_PREFIX}.250"



### Helper Functions ###

# Check that we can get sudo privs
test_sudo()
{
    sudo echo &> /dev/null
    if [ $? != 0 ]; then
        echo "Failed to gain sudo privs!"
        exit 1
    fi
}

# Note: requires root
setup_bridge()
{
    test_sudo

    # Check that the bridge device exists; if not, create it
    ip link show dev $BRIDGE_DEV &> /dev/null
    if [ $? != 0 ]; then
        echo "Building software bridge ..."
        sudo ip link add $BRIDGE_DEV type bridge
        sudo ip addr add $BRIDGE_IP/$NET_MASK broadcast $NET_BCAST dev $BRIDGE_DEV
    fi

    # Always make sure it is set to 'up'
    sudo ip link set dev $BRIDGE_DEV up
}

