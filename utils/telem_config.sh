#!/bin/bash

# Set defaults and constant values
PLANE_CONFIG=false
PLANE_RECONFIG=false
BAUDRATE=57600
ATC=$ACS_ROOT/SiK/Firmware/tools/atcommander.py
MAV=$ACS_ROOT/acs_ros_ws/src/autonomy-payload/utils/mavlink_rate.py
NETID=25
MINFREQ=915000
MAXFREQ=928000

# Functions

set_local()
{
    echo ""
    echo "Reconfiguring local radio ..."
    echo ""
    $ATC -f -L netid $NETID -L minfreq $MINFREQ -L maxfreq $MAXFREQ $DEVICE
    if [ $? != 0 ]; then
        echo ""
        echo "Failed to configure local radio to defaults"
        echo ""
        exit 1
    fi
}

set_remote_then_local()
{
    echo ""
    echo "Reconfiguring remote radio and then local radio ..."
    echo ""
    $ATC -R netid $NETID -R minfreq $MINFREQ -R maxfreq $MAXFREQ \
         -L netid $NETID -L minfreq $MINFREQ -L maxfreq $MAXFREQ \
         -f $DEVICE
    if [ $? != 0 ]; then
        echo ""
        echo "Failed to configure radios to new settings"
        echo ""
        exit 1
    fi
}

check_traffic()
{
    echo ""
    echo "Checking for traffic from plane (may hang indefinitely) ..."
    echo ""
    $MAV $DEVICE $BAUDRATE 0
    if [ $? != 0 ]; then
        echo ""
        echo "Failed to observe traffic from plane"
        echo ""
        exit 1
    fi
}

usage()
{
cat <<EOF
Usage: $0 [options] telem-device new-netid
Options:
    -B BAUDRATE     Baudrate to use (default $BAUDRATE)
    -C              Configure a new plane's radio to new-netid in sub-band
    -N NETID        Current Net ID of plane's radio (default $NETID)
    -R              Reconfigure a pre-configured plane's radio
    -h              Show this help and exit

Configures local radio to talk to a plane (default behavior)
Use -C to configure a brand-new plane's radio
Use -R with -N to re-configure a plane's radio to a new netid
Use -C with -N to re-configure a plane's radio if in old half-band
EOF
}

# Argument parsing

# Parse optional args
while getopts ":B:CN:Rh" opt; do
    case $opt in
        B)
            BAUDRATE=$OPTARG
            ;;
        C)
            PLANE_CONFIG=true
            ;;
        N)
            NETID=$OPTARG
            ;;
        R)
            PLANE_RECONFIG=true
            ;;
        *)
            usage
            exit 0
            ;;
    esac
done
shift $((OPTIND-1))

# Parse positional args
if [ -z $1 ] || [ -z $2 ]; then
    usage
    exit 1
fi
DEVICE=$1
NETID_NEW=$2

# Check mutually-exlusive options
if $PLANE_CONFIG && $PLANE_RECONFIG; then
    usage
    exit 1
fi

# Do the configuration

# If reconfiguring a plane, use its current sub-band
if $PLANE_RECONFIG; then
    MINFREQ=$(($NETID % 3 * 9000 + 902000))
    MAXFREQ=$(($NETID % 3 * 9000 + 910000))
fi

# If (re)configuring a plane, find it first
if $PLANE_CONFIG || $PLANE_RECONFIG; then
    # Reset local radio to match plane
    set_local
    # Verify that we see traffic from plane
    check_traffic
fi

# Calculate updated radio settings
NETID=$NETID_NEW
MINFREQ=$(($NETID % 3 * 9000 + 902000))
MAXFREQ=$(($NETID % 3 * 9000 + 910000))

# Reconfigure radio(s) as needed
if $PLANE_CONFIG || $PLANE_RECONFIG; then
    # Reset plane's radio, then local radio
    set_remote_then_local
    # Verify that we see traffic from plane
    check_traffic
else
    # Reset local radio only
    set_local
fi

# If everything succeeded, report and exit
echo ""
echo "SUCCESS"
echo ""
exit 0

