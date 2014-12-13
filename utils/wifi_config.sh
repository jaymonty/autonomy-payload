#!/bin/sh

if [[ -z $1 || -z $2 ]]; then
  echo "usage: $0 DEVICE ID-NUMBER"
fi

ifconfig $1 down
iwconfig $1 mode ad-hoc essid zephyr channel 6 ap 00:11:22:33:44:55
ifconfig $1 inet 192.168.2.$2/24 up

