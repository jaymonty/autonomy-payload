#!/bin/bash

# Read in the ID for the virtual aircraft. THIS MUST BE UNIQUE.
# Note that this MUST be the same as the SITL "-I" value.

#process options first:

usage()
{
cat <<EOF
Usage: $0 [options] instance_id
Options:
    -C                      Start a linux container for this payload SITL
    -R                      Username to run ROS as (container mode only)
EOF
}

USE_CONTAINER=0
ROS_USER_TO_USE=$USER

#parse options
while getopts ":R:Ch" opt; do
    case $opt in
        C)
            USE_CONTAINER=1
            ;;
        R)
            if [[ `getent passwd | grep -c "^$OPTARG"` != 0 ]]; then
                ROS_USER_TO_USE=$OPTARG
            else
                echo "ERROR. Unknown user: $OPTARG -- unable to launch payload."
                exit -33
            fi
            ;;
        h)
            usage
            exit 0
            ;;
    esac
done
shift $((OPTIND-1))

if [[ -z $1 ]]; then
  echo "Usage: $0 [options] instance_id [ROS_username]"
  exit 1
fi

ID=$1
shift 1  # Shift in case we want to send $@ elsewhere later on

# This variable should be set in newer SITL installs, but if it isn't
# then default to the user's home directory

if [[ -z $ACS_ROOT ]]; then
  ACS_ROOT="~"
fi


if [ $USE_CONTAINER == 0 ]; then
    SITL_CONNECT="tcp:127.0.0.1:"$((5762+10*${ID}))
    PORT="$((5554+${ID}))"   

# Launch a terminal from within the new namespace
    roslaunch ap_master sitl_ns.launch id:=$ID name:=sitl$ID sitl:=$SITL_CONNECT port:=$PORT ns:=sitl$ID

else 
    # Set up the basic networking parameters for our multi-SITL environment.
    # This is set to mimic our flight configuration as closely as possible.
    # All aircraft (and ground stations) are in 192.168.2.0/24

    NET_IP_PREFIX="192.168.2"
    NET_MASK="255.255.255.0"
    NET_BCAST="${NET_IP_PREFIX}.255"

    # New network namespaces need unique names, and new network interfaces
    # need namespace-unique names. These are the prefixes; a number will
    # be appended to each where appropriate.

    NET_DEV_PREFIX="sitl"
    NET_NS_PREFIX="sitl"

    # We create a software bridge in the original namespace so the base
    # machine can communicate with virtual aircraft. It has a reserved
    # IP address.

    BRIDGE_DEV="sitl_bridge"
    BRIDGE_IP="${NET_IP_PREFIX}.250"

    # Compute the MAVLink connection string (e.g., tcp:192.168.2.250:5772)
    SITL_CONNECT="tcp:${BRIDGE_IP}:"$((5762+10*${ID}))

    # Define the instance-specific networking parameters. Each virtual aircraft
    # gets its own network namespace and a new virtual network interface.
    # Currently, the virtual interface is renamed inside the namespace to "eth0"
    # for consistency. In the original namespace, the corresponding interface
    # is named "sitlN" for ID N.
    SITL_IP="${NET_IP_PREFIX}.${ID}"
    SITL_NS="${NET_NS_PREFIX}${ID}"
    SITL_DEV="${NET_DEV_PREFIX}${ID}_br"
    SITL_DEV_ALIAS="eth0"
    SITL_DEV_BR="${NET_DEV_PREFIX}${ID}"

    # Check that the bridge device exists; if not, create it
    ip link show dev $BRIDGE_DEV &> /dev/null
    if [ $? != 0 ]; then
        echo "Building software bridge ..."
        sudo ip link add $BRIDGE_DEV type bridge
        sudo ip addr add $BRIDGE_IP/$NET_MASK broadcast $NET_BCAST dev $BRIDGE_DEV
    fi
    # Always make sure it is set to 'up'
    sudo ip link set dev $BRIDGE_DEV up

    echo "Building a new network namespace configuration ..."

    # Create a virtual interface pair. One side will remain in the original
    # namespace and attach to the software bridge; the other will be moved
    # into a new namespace.
    sudo ip link add dev $SITL_DEV type veth peer name $SITL_DEV_BR

    # Set up the bridge side of the pair
    sudo ip link set dev $SITL_DEV_BR up
    sudo ip link set $SITL_DEV_BR master $BRIDGE_DEV

    # Create a new network namespace and set up its networking
    sudo ip netns add $SITL_NS
    sudo ip link set $SITL_DEV netns $SITL_NS
    sudo ip netns exec $SITL_NS \
        ip link set dev lo up
    sudo ip netns exec $SITL_NS \
        ip link set dev $SITL_DEV name $SITL_DEV_ALIAS
    sudo ip netns exec $SITL_NS \
        ip addr add $SITL_IP/$NET_MASK broadcast $NET_BCAST dev $SITL_DEV_ALIAS
    sudo ip netns exec $SITL_NS \
        ip link set dev $SITL_DEV_ALIAS up

    echo "Launching payload in the new namespace ..."

    # Launch a terminal from within the new namespace
    sudo ip netns exec $SITL_NS \
        su -l -c "source /opt/ros/$ROS_DISTRO/setup.bash; source $ACS_ROOT/acs_ros_ws/devel/setup.bash; roslaunch ap_master sitl.launch id:=$ID name:=$SITL_NS dev:=$SITL_DEV_ALIAS sitl:=$SITL_CONNECT" $ROS_USER_TO_USE

    echo ""
    echo "Tearing down the namespace (may require sudo password) ..."

    # When terminal exits, tear down the virtual interface pair and namespace
    sudo ip link del dev $SITL_DEV_BR
    sudo ip netns del $SITL_NS
fi

