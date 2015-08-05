#!/bin/bash

BAUDRATE=57600
ATC=$ACS_ROOT/SiK/Firmware/tools/atcommander.py
MAV=$ACS_ROOT/acs_ros_ws/src/autonomy-payload/utils/mavlink_rate.py
NETID_OLD=25
MINFREQ_OLD=915000
MAXFREQ_OLD=928000

usage()
{
cat <<EOF
Usage: $0 [options] device new-netid
Options:
    -B BAUDRATE     Baudrate to use (default $BAUDRATE)
    -N NETID        Initial netid to use (default $NETID_OLD)
    -S              Reset existing netid into new sub-bands
EOF
}

while getopts ":B:N:Sh" opt; do
    case $opt in
        B)
            BAUDRATE=$OPTARG
            ;;
        N)
            NETID_OLD=$OPTARG
            ;;
        S)
            NETID_OLD=-1
            ;;
        h)
            usage
            exit 0
            ;;
    esac
done
shift $((OPTIND-1))

if [ -z $1 ] || [ -z $2 ]; then
    usage
    exit 1
fi

DEVICE=$1
NETID_NEW=$2
MINFREQ_NEW=$(($NETID_NEW % 3 * 9000 + 902000))
MAXFREQ_NEW=$(($NETID_NEW % 3 * 9000 + 910000))

if [ $NETID_OLD == -1 ]; then
    NETID_OLD=$NETID_NEW
fi

echo ""
echo "Resetting local radio ..."
echo ""
$ATC -f -L netid $NETID_OLD -L minfreq $MINFREQ_OLD -L maxfreq $MAXFREQ_OLD $DEVICE
if [ $? != 0 ]; then
    echo ""
    echo "Failed to configure local radio to defaults"
    echo ""
    exit 1
fi

echo ""
echo "Looking for traffic from plane ..."
echo ""
$MAV $DEVICE $BAUDRATE 0
if [ $? != 0 ]; then
    echo ""
    echo "Failed to observe traffic from plane"
    echo ""
    exit 1
fi

echo ""
echo "Reconfiguring both radios to new settings ..."
echo ""
$ATC -f -R netid $NETID_NEW -R minfreq $MINFREQ_NEW -R maxfreq $MAXFREQ_NEW \
     -f -L netid $NETID_NEW -L minfreq $MINFREQ_NEW -L maxfreq $MAXFREQ_NEW \
     $DEVICE
if [ $? != 0 ]; then
    echo ""
    echo "Failed to configure radios to new settings"
    echo ""
    exit 1
fi

echo ""
echo "Verifying traffic from reconfigured plane ..."
echo ""
$MAV $DEVICE $BAUDRATE 0
if [ $? != 0 ]; then
    echo ""
    echo "Failed to observe traffic from plane"
    echo ""
    exit 1
fi

echo ""
echo "SUCCESS"
echo ""
exit 0

