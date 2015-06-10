#!/bin/bash

# Script to deploy necessary files onto an eMMC (or SD) card
# with a fresh Ubuntu install, such as those we purchase from
# the factory.

TARGET=$1
VERSION=$2
BOOT_NUM=1
ROOT_NUM=2
TARGET_BOOT=${TARGET}${BOOT_NUM}
TARGET_ROOT=${TARGET}${ROOT_NUM}
MOUNTPOINT=/media/odroid
HOMEDIR=$MOUNTPOINT/home/odroid
SCRIPT=odroid-installer-${VERSION}.sh
NEWSCRIPT=odroid-installer.sh

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

if [ -z $TARGET ] || [ -z $VERSION ]; then
  echo "usage: $0 <target-device> <version>"
  exit 1
fi

if [ ! -e $SCRIPT ]; then
  echo "You must run $0 from a folder containing $SCRIPT."
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

# Attempt to make sure card is not mounted
umount $TARGET_BOOT &> /dev/null
umount $TARGET_ROOT &> /dev/null

echo "Preparing media for installation ..."

./expand.sh $TARGET $ROOT_NUM
check_fail "Couldn't prepare the media."

echo "Deploying installer script ..."

mount $TARGET_ROOT $MOUNTPOINT
check_fail "Couldn't mount the filesystem."

cp $SCRIPT $HOMEDIR/$NEWSCRIPT
check_fail "Couldn't copy installer to home directory."
chown 1001:1001 $HOMEDIR/$NEWSCRIPT
check_fail "Couldn't set ownership on installer script."
chmod +x $HOMEDIR/$NEWSCRIPT
check_fail "Couldn't set permissions on installer script."

umount $MOUNTPOINT
check_fail "Couldn't umount filesystem."

echo ""
echo "The media is prepared for installation. Please boot it on an ODroid"
echo "using a console cable:"
echo "    miniterm.py /dev/ttyUSB0 115200 --lf"
echo "then perform the following:"
echo "    su - odroid"
echo "    ./odroid-installer.sh"
echo "and follow the prompts. If you are using older media that boots to"
echo "a configuration screen, simply exit to a root prompt first."
echo ""

