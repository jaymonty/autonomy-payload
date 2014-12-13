#!/bin/bash

# Purpose: Start up a payload instance
# Author: Mike Clement



### Configuration Variables ###

# This variable should be set in newer SITL installs, but if it isn't
# then default to the user's home directory

if [[ -z $ACS_ROOT ]]; then
    ACS_ROOT="~"
fi

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
    # Check that the bridge device exists; if not, create it
    ip link show dev $BRIDGE_DEV &> /dev/null
    if [ $? != 0 ]; then
        echo "Building software bridge ..."

        test_sudo

        sudo ip link add $BRIDGE_DEV type bridge
        sudo ip addr add $BRIDGE_IP/$NET_MASK broadcast $NET_BCAST dev $BRIDGE_DEV
    fi

    # Always make sure it is set to 'up'
    sudo ip link set dev $BRIDGE_DEV up
}



### Process Command-Line Options ###

usage()
{
cat <<EOF
Usage: $0 [options] instance_id
Options:
    -B              Set up and use SITL bridge (implies '-D ${BRIDGE_DEV}')
    -C              Use Linux network container (don't use with -P; implies -B)
    -D DEVICE       Override default network device (don't use with -C)
    -P              Use calculated port offset for network (don't use with -C)
    -R USER         Username to run ROS as (use with -C only)
EOF
}

# option defaults
USE_CONTAINER=0
USE_PORT_OFFSET=0
ROS_USER_TO_USE=$USER
NET_DEVICE="lo"

#parse options
while getopts ":BCD:PR:h" opt; do
    case $opt in
        B)
            NET_DEVICE=$BRIDGE_DEV
            ;;
        C)
            if [ $USE_PORT_OFFSET == 1 ]; then
                echo "ERROR. Cannot use -C with -P."
                exit 1
            fi
            USE_CONTAINER=1
            NET_DEVICE=$BRIDGE_DEV
            ;;
        D)
            NET_DEVICE=$OPTARG
            ;;
        P)
            if [ $USE_CONTAINER == 1 ]; then
                echo "ERROR. Cannot use -C with -P."
                exit 1
            fi
            USE_PORT_OFFSET=1
            ;;
        R)
            if [[ `getent passwd | grep -c "^$OPTARG"` != 0 ]]; then
                ROS_USER_TO_USE=$OPTARG
            else
                echo "ERROR. Unknown user: $OPTARG -- unable to launch payload."
                exit 1
            fi
            ;;
        h)
            usage
            exit 0
            ;;
    esac
done
shift $((OPTIND-1))

# test for positional parameter
if [[ -z $1 ]]; then
    usage
    exit 1
fi

# get positional parameter
ID=$1
shift 1  # Shift in case we want to send $@ elsewhere later on



### Calculate IPs, Ports, etc ###

# IP and port for connection to SITL autopilot
SITL_PORT=$((5762+10*${ID}))
SITL_IP=`ip addr show dev $NET_DEVICE | grep 'inet ' | cut -d ' ' -f 6 | cut -d '/' -f 1`
if [ $? != 0 ]; then
    echo "Could not get IP for $NET_DEVICE"
    exit 1
fi
SITL_CONNECT="tcp:${SITL_IP}:${SITL_PORT}"

# Port for network comms
NET_PORT=5554
if [ $USE_PORT_OFFSET == 1 ]; then
    NET_PORT=$((5554+${ID}))
fi



### If needed, set up bridge ###

if [ $NET_DEVICE == $BRIDGE_DEV ]; then
    setup_bridge
fi



### Handle Startup Cases ###

# Non-container case
if [ $USE_CONTAINER == 0 ]; then
    roslaunch ap_master sitl.launch id:=$ID name:=sitl$ID sitl:=$SITL_CONNECT port:=$NET_PORT ns:=sitl$ID dev:=$NET_DEVICE
fi

# Container case
# NOTE: We don't run inside a ROS namespace because the containers don't see each other
if [ $USE_CONTAINER == 1 ]; then
    # Define the instance-specific networking parameters. Each virtual aircraft
    # gets its own network namespace and a new virtual network interface.
    # Currently, the virtual interface is renamed inside the namespace to "eth0"
    # for consistency. In the original namespace, the corresponding interface
    # is named "sitlN" for ID N.
    PAYL_IP="${NET_IP_PREFIX}.${ID}"
    PAYL_NS="${NET_NS_PREFIX}${ID}"
    PAYL_DEV="${NET_DEV_PREFIX}${ID}_br"
    PAYL_DEV_ALIAS="eth0"
    PAYL_DEV_BR="${NET_DEV_PREFIX}${ID}"

    echo "Building a new network namespace configuration ..."
    test_sudo  # Should be done by setup_bridge above

    # Create a virtual interface pair. One side will remain in the original
    # namespace and attach to the software bridge; the other will be moved
    # into a new namespace.
    sudo ip link add dev $PAYL_DEV type veth peer name $PAYL_DEV_BR

    # Set up the bridge side of the pair
    sudo ip link set dev $PAYL_DEV_BR up
    sudo ip link set $PAYL_DEV_BR master $BRIDGE_DEV

    # Create a new network namespace and set up its networking
    sudo ip netns add $PAYL_NS
    sudo ip link set $PAYL_DEV netns $PAYL_NS
    sudo ip netns exec $PAYL_NS \
        ip link set dev lo up
    sudo ip netns exec $PAYL_NS \
        ip link set dev $PAYL_DEV name $PAYL_DEV_ALIAS
    sudo ip netns exec $PAYL_NS \
        ip addr add $PAYL_IP/$NET_MASK broadcast $NET_BCAST dev $PAYL_DEV_ALIAS
    sudo ip netns exec $PAYL_NS \
        ip link set dev $PAYL_DEV_ALIAS up

    echo "Launching payload in the new namespace ..."

    # Launch a terminal from within the new namespace
    sudo ip netns exec $PAYL_NS \
        su -l -c "source /opt/ros/$ROS_DISTRO/setup.bash; source $ACS_ROOT/acs_ros_ws/devel/setup.bash; roslaunch ap_master sitl.launch id:=$ID name:=$PAYL_NS dev:=$PAYL_DEV_ALIAS sitl:=$SITL_CONNECT" $ROS_USER_TO_USE

    echo ""
    echo "Tearing down the namespace (may require sudo password) ..."

    # When terminal exits, tear down the virtual interface pair and namespace
    test_sudo
    sudo ip link del dev $PAYL_DEV_BR
    sudo ip netns del $PAYL_NS
fi

