#!/bin/bash

DEV_NS=$1
DEV_IFACE=$2
DEV_IP=$3
DEV_MASK=$4
DEV_BCAST=$5

shift 5

DEV_IFACE_BR="${DEV_IFACE}_br"
BR_IFACE="sitl_bridge"

# Check that the bridge device exists
ip link show dev $BR_IFACE &> /dev/null
if [ $? != 0 ]; then
  sudo ip link add $BR_IFACE type bridge
  sudo ip link set dev $BR_IFACE up
fi

# Create the virtual interface pair
sudo ip link add dev $DEV_IFACE type veth peer name $DEV_IFACE_BR

# Set up the bridged side of the pair
sudo ip link set dev $DEV_IFACE_BR up
sudo ip link set $DEV_IFACE_BR master $BR_IFACE

# Create a network namespace and move the device into it
sudo ip netns add $DEV_NS
sudo ip link set $DEV_IFACE netns $DEV_NS
sudo ip netns exec $DEV_NS ip link set dev lo up

# Set up the device side of the pair in the namespace
sudo ip netns exec $DEV_NS \
  ip addr add $DEV_IP/$DEV_MASK broadcast $DEV_BCAST dev $DEV_IFACE
sudo ip netns exec $DEV_NS \
  ip link set dev $DEV_IFACE up

# Launch the payload from within the new namespace
sudo ip netns exec $DEV_NS \
  xterm -e su - $USER

# Tear down the virtual interface pair and namespace
sudo ip link del dev $DEV_IFACE_BR
sudo ip netns del $DEV_NS


