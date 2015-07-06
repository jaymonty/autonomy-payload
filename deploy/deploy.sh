#!/bin/bash

# Script to expand a filesystem and deploy an installation script
# onto media (eMMC, SD) for an ODroid.

# If any operation fails, just exit so user can fix the problem
check_fail()
{
  if [ $? != 0 ]; then
    echo -e "\nERROR: $1\n"
    kill -INT $$
  fi
}

do_deploy()
{
  # Attempt to make sure card is not mounted
  umount $TARGET_BOOT &> /dev/null
  umount $TARGET_ROOT &> /dev/null

  if [ $CLONE_ACT == 1 ]; then
    # Clone media from a file
    echo "Cloning $CLONE_FILE onto $TARGET, this may take a few minutes ..."
    dd if=$CLONE_FILE of=$TARGET bs=4M
    check_fail "Couldn't clone the media."
  fi

  # Run script that expands the partition and filesystem
  echo "Preparing media ..."
  ./expand.sh $TARGET $ROOT_NUM
  check_fail "Couldn't prepare the media."

  if [ $INSTALLER_ACT == 1 ]; then
    # Copy the latest installer onto the filesystem
    echo "Deploying installer script ..."
    mount $TARGET_ROOT $MOUNTPOINT
    check_fail "Couldn't mount the filesystem."
    if [ -e $NEWSCRIPT ] || [ -L $NEWSCRIPT ]; then
      echo "Removing old installer ..."
      rm -f $NEWSCRIPT
      check_fail "Couldn't remove old installer."
    fi
    cp $INSTALLER_FILE $NEWSCRIPT
    check_fail "Couldn't copy installer to home directory."
    chown 1001:1001 $NEWSCRIPT
    check_fail "Couldn't set ownership on installer script."
    chmod +x $NEWSCRIPT
    check_fail "Couldn't set permissions on installer script."
    umount $MOUNTPOINT
    check_fail "Couldn't umount filesystem."
  fi
}

usage()
{
cat <<EOF
Usage: $0 [options] device
Options:
    -C FILE     Clone to device from FILE
    -I VERSION  Place install script in home folder of device
EOF
}

#-----------------------------------------------------------------------------
# Main program

# Initialize constants and defaults
BOOT_NUM=1                          # Partition numbers
ROOT_NUM=2
MOUNTPOINT=/media/odroid            # Where we'll mount it
HOMEDIR=$MOUNTPOINT/home/odroid     # Where installer goes
SCRIPT_SHORT=odroid-installer.sh    # Shortened name of script
NEWSCRIPT=$HOMEDIR/$SCRIPT_SHORT    # Name of placed installer
CLONE_ACT=0                         # 1 = clone media from image
CLONE_FILE=""                       # Location of image
INSTALLER_ACT=0                     # 1 = place installer file
INSTALLER_FILE=""                   # Filename of installer to place

# Parse arguments
while getopts ":C:I:h" opt; do
  case $opt in
    C)
      if [ -z OPTARG ]; then
        usage
        exit 1
      fi
      CLONE_ACT=1
      CLONE_FILE=$OPTARG
      ;;
    I)
      if [ -z OPTARG ]; then
        usage
        exit 1
      fi
      INSTALLER_ACT=1
      INSTALLER_FILE=odroid-installer-${OPTARG}.sh
      ;;
    h)
      usage
      exit 0
      ;;
  esac
done
shift $((OPTIND-1))

# Get target device
TARGET=$1
TARGET_BOOT=${TARGET}${BOOT_NUM}    # Partition paths
TARGET_ROOT=${TARGET}${ROOT_NUM}

# Sanity checks
if [ `whoami` != "root" ]; then
  echo "You must run this script as root."
  exit 1
fi
if [ -z $TARGET ]; then
  usage
  exit 1
fi
if [ $CLONE_ACT == 1 ] && [ ! -e $CLONE_FILE ]; then
  echo "Cannot find $CLONE_FILE"
  exit 1
fi
if [ $INSTALLER_ACT == 1 ] && [ ! -e $INSTALLER_FILE ]; then
  echo "You must run $0 from a folder containing $INSTALLER_FILE"
  exit 1
fi

# Set up a mountpoint we can use
mkdir $MOUNTPOINT &> /dev/null
ls $MOUNTPOINT &> /dev/null
check_fail "Couldn't create $MOUNTPOINT"
mountpoint $MOUNTPOINT &> /dev/null
if [ $? == 0 ]; then
  echo "Warning: unmounting something else from $MOUNTPOINT"
  umount $MOUNTPOINT
  check_fail "Couldn't umount $MOUNTPOINT"
fi

# Run in a loop until users quits
REPLY="y"
while [[ $REPLY =~ ^[Yy]$ ]]; do
  # Wait for media to be present (must be a block device)
  echo "Please plug in $TARGET (if not done already) ..."
  while [ ! -b $TARGET ]; do
    sleep 0.5
  done

  # Perform deployment operations
  do_deploy

  # Make sure user unplugs card before we re-run
  echo "Media is prepared; please unplug it from your system now."
  while [ -b $TARGET ]; do
    sleep 0.5
  done

  # See if user wants to go again
  echo ""
  read -p "Run again (y/n)? " -n 1 -r
  echo ""
done

# Give final instructions
cat <<EOF

Thank you for using the ACS ODroid deployment script.

To update or install the ACS software suite, boot the media on an ODroid,
then either SSH in (if IP is known and using the ACS wireless network):

    ssh odroid@192.168.2.xxx

or connect using a console cable (changing 'ttyUSB0' as needed):

    miniterm.py /dev/ttyUSB0 115200 --lf

then perform the following (su only if logged in as root):

    su - odroid
    ./odroid-installer.sh [-u | -q]

and follow the prompts. If you are using older media that boots to
a configuration screen, simply exit to a root prompt first.

EOF

