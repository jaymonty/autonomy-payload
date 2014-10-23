#!/bin/bash

# Script to deploy necessary files onto an eMMC (or SD) card
# with a fresh Ubuntu install, such as those we purchase from
# the factory.

TARGET=$1
TARGET_ROOT=${TARGET}2
MOUNTPOINT=/media/odroid
HOMEDIR=$MOUNTPOINT/home/odroid
SCRIPT=odroid-installer.sh

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

if [ ! -e $SCRIPT ]; then
  echo "You must run $0 from a folder containing $SCRIPT."
  exit 1
fi

if [ -z $TARGET ]; then
  echo "usage: $0 <target-device>"
  exit 1
fi

mkdir $MOUNTPOINT &> /dev/null
ls $MOUNTPOINT &> /dev/null
check_fail "Couldn't create $MOUNTPOINT"
mountpoint $MOUNTPOINT &> /dev/null
if [ $? == 0 ]; then
  echo "Something is already mounted at $MOUNTPOINT; please unmount it first."
  exit 1
fi

echo "Preparing media for installation ..."

# NOTE: This was copied and modified from the old odroid-config utility
p2_start=`fdisk -l $TARGET | grep $TARGET_ROOT | awk '{print $2}'`
fdisk $TARGET <<EOF
p
d
2
n
p
2
$p2_start

p
w
EOF
check_fail "Couldn't resize the partition."

e2fsck -f -y $TARGET_ROOT
resize2fs $TARGET_ROOT
check_fail "Couldn't expand the filesystem."

mount $TARGET_ROOT $MOUNTPOINT
check_fail "Couldn't mount the filesystem."

cp $SCRIPT $HOMEDIR
check_fail "Couldn't copy installer to home directory."
chmod +x $HOMEDIR/$SCRIPT
check_fail "Couldn't set permissions on installer script."

umount $MOUNTPOINT
check_fail "Couldn't umount filesystem."

echo "The media is prepared for installation. Please boot it on an ODroid"
echo "using a console cable, then perform the following:"
echo "    su - odroid"
echo "    ./odroid-installer.sh"
echo "and follow the prompts. If you are using older media that boots to"
echo "a configuration screen, simply exit to a root prompt first."
echo ""

