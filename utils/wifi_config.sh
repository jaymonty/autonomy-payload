#!/bin/bash

usage()
{
cat <<EOF
Usage: $0 [options] wireless-device id-number
Options:
    -R DEVICE       Set up as router, with DEVICE facing the world
    -T POWER        Set txpower (default=10)
EOF
}

# option defaults
ROUTER_USE=0
ROUTER_DEV=""
TXPOWER=10

#parse options
while getopts ":R:T:h" opt; do
    case $opt in
        R)
            if [ -z $OPTARG ]; then
                usage
                exit 1
            fi
            ROUTER_USE=1
            ROUTER_DEV=$OPTARG
            ;;
        T)
            if [ -z $OPTARG ]; then
                usage
                exit 1
            fi
            TXPOWER=$OPTARG
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

if [ $ROUTER_USE != 0 ]; then
  echo "Setting up router ..."
  sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"
  sudo iptables -t nat -F  # Flush old rules first
  sudo iptables -t nat -A POSTROUTING -o $ROUTER_DEV -j MASQUERADE
fi

# Make sure SITL bridge device isn't running
/sbin/ip link | grep sitl_bridge | grep -q ',UP,'
if [ $? == 0 ]; then
  echo "Disabling sitl_bridge interface ..."
  sudo ifconfig sitl_bridge down
fi

sudo ifconfig $1 down
sudo iwconfig $1 mode ad-hoc essid zephyr channel 6 \
  ap 00:11:22:33:44:55 txpower $TXPOWER
sudo ifconfig $1 inet 192.168.2.$2/24 up

