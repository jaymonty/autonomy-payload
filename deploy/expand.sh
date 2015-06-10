#!/bin/bash

# Script to expand a partition and EXT* filesystem

DEVICE=$1
PARTNUM=$2
DEVPART=$1$2

function check_fail
{
  if [ $? != 0 ]; then
    echo -e "\nERROR: $1\n"
    kill -INT $$
  fi
}

if [ `whoami` != "root" ]; then
  echo "You must run this script as root."
  exit 1
fi

if [ -z $DEVICE ] || [ -z $PARTNUM ]; then
  echo "usage: $0 <target-device> <partition-number>"
  exit 1
fi

# Make sure partition is unmounted first
umount $DEVPART &> /dev/null

# Expand the partition to the maximum size
echo "Resizing the partition ..."
part_start=`fdisk -l $DEVICE | grep $DEVPART | awk '{print $2}'`
if [ -z $part_start ]; then
  echo "Could not determine partition start sector."
  exit 1
fi
fdisk $DEVICE <<EOF &> /dev/null
p
d
2
n
p
2
$part_start

p
w
EOF
check_fail "Couldn't resize the partition."

# Requires a filesystem check after partition change
echo "Checking the filesystem ... please wait ..."
e2fsck -f -y $DEVPART &> /dev/null
if [ $? != 0 -a $? != 1 -a $? != 2 ]; then
  echo "Irrecoverable fsck errors ($?); please run manually"
  exit 1
fi

# Resize the filesystem inside the partition
echo "Resizing the filesystem ..."
resize2fs $DEVPART &> /dev/null
check_fail "Couldn't expand the filesystem."

exit 0

