#!/bin/bash

#------------------------------------------------------------------------------
# Purpose: Set up autonomy-payload on SITL or development computer,
#          assuming standard Ubuntu install and SITL install
# NOTE: Set up for a Hydro install on an x86 or x86_64 machine
# Author: Mike Clement
#------------------------------------------------------------------------------

# We want to run as the actual user, not as root
if [ `whoami` == "root" ]; then
  echo "Please run as the user that will run the payload software."
  exit 1
fi

# Unusual, but might not have all SITL components installed and need mavlink lib
echo ""
echo "All systems require the mavlink library, which is installed as part of SITL."
echo "Rarely, this script may be used on a system without a SITL environment."
echo "Answer 'y' ONLY IF a SITL has not been installed by this user on this computer."
echo ""
read -p "Does this computer need mavlink installed [y/N]? " INSTALL_MAVLINK
if [ -z $INSTALL_MAVLINK ]; then INSTALL_MAVLINK='n'; fi

# Handle development (push-able) vs SITL (pull-only) cases
echo ""
read -p "Will git commits be pushed from this computer [y/N]? " GIT_PUSHABLE
if [ -z $GIT_PUSHABLE ]; then GIT_PUSHABLE='n'; fi
if [ $GIT_PUSHABLE == 'y' ]; then
  echo ""
  read -p "Please make sure this computer's SSH key is set up in Gitlab, then press Enter."
fi

#------------------------------------------------------------------------------
# General setup

# Function to check for command failure and print useful error messages
function check_fail
{
  if [ $? != 0 ]; then
    echo -e "\nERROR: $1\n"
    exit 1
  fi
}

# Regrettably, we need to disable Git's SSL cert check for HTTP connections
if [ $GIT_PUSHABLE != 'y' ]; then
  git config --global http.sslVerify false
fi

#------------------------------------------------------------------------------
# Install packages 

# Update locale settings
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

# Add ROS repo
ls /etc/apt/sources.list.d/ros-latest.list &> /dev/null
if [ $? != 0 ]; then
  sudo sh -c 'wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -'
  check_fail "ROS apt-key"
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
  check_fail "ROS apt source"
fi

# Update package lists
sudo apt-get update
check_fail "apt-get update"

# Upgrade existing packages
sudo apt-get --assume-yes upgrade
check_fail "apt-get upgrade"

# Install ROS and other useful packages
sudo apt-get --assume-yes install \
ros-hydro-desktop-full \
ros-hydro-sensor-msgs \
ros-hydro-robot-pose-ekf \
python-netifaces \
python-setuptools
check_fail "apt-get install"

#------------------------------------------------------------------------------
# Set up ROS

sudo rosdep init
rosdep update
grep "/opt/ros/hydro/setup.bash" ~/.bashrc
if [ $? != 0 ]; then
  echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
fi
source /opt/ros/hydro/setup.bash

#------------------------------------------------------------------------------
# Install (py)mavlink library

if [[ $INSTALL_MAVLINK == 'y' ]]; then
  cd ~

  ls mavlink/ &> /dev/null
  if [ $? != 0 ]; then
    if [ $GIT_PUSHABLE == 'y' ]; then
      git clone git@yoda:aerial-combat-swarms/mavlink.git
      check_fail "mavlink git clone (ssh)"
    else
      git clone https://yoda.ern.nps.edu:18080/aerial-combat-swarms/mavlink.git
      check_fail "mavlink git clone (http)"
    fi
    cd mavlink/pymavlink/
    git checkout dev
    check_fail "mavlink git checkout dev"

    python setup.py build install --user
    check_fail "mavlink setup.py"
  else
    cd mavlink/pymavlink/
    git checkout .  # reset to state where we can update
    check_fail "mavlink git checkout ."
    git fetch origin  # update local copy of repo
    check_fail "mavlink git fetch"
    git diff --quiet origin/dev  # determine if there are updates
    MAVLINK_REBUILD=$?
    git checkout dev  # bring updates into working space
    check_fail "mavlink git checkout dev"

    # Only rebuild if the branch was actually updated
    if [ $MAVLINK_REBUILD != 0 ]; then
      python setup.py build install --user
      check_fail "mavlink setup.py"
    fi
  fi
fi

#------------------------------------------------------------------------------
# Set up autonomy-payload

# Initialize the ROS workspace
# (highly unlikely we'll see failures here; ignoring check_fail())
cd ~
ls acs_ros_ws/src/ &> /dev/null
if [ $? != 0 ]; then
  mkdir -p acs_ros_ws/src/
  cd acs_ros_ws/src/
  catkin_init_workspace
  cd ..
  catkin_make
  echo "source ~/acs_ros_ws/devel/setup.bash" >> ~/.bashrc
fi
source ~/acs_ros_ws/devel/setup.bash

# Clone or update the autonomy-payload repo
cd ~/acs_ros_ws/src/
ls autonomy-payload/ &> /dev/null
if [ $? != 0 ]; then
  if [ $GIT_PUSHABLE == 'y' ]; then
    git clone git@yoda:aerial-combat-swarms/autonomy-payload.git
    check_fail "payload git clone (ssh)"
  else
    git clone https://yoda:18080/aerial-combat-swarms/autonomy-payload.git
    check_fail "payload git clone (http)"
  fi
else
  cd autonomy-payload
  git checkout .  # reset to state where we can update
  check_fail "payload git checkout ."
  git fetch origin  # update local copy of repo
  check_fail "payload git fetch"
  git checkout master  # bring updates into working space
  check_fail "payload git checkout dev"
fi

# Install ACS shared libs
cd ~/acs_ros_ws/src/autonomy-payload/lib/
python setup.py build install --user
check_fail "payload lib setup.py"

# Clone or update the autopilot_bridge repo
cd ~/acs_ros_ws/src/
ls autopilot_bridge/ &> /dev/null
if [ $? != 0 ]; then
  if [ $GIT_PUSHABLE == 'y' ]; then
    git clone git@yoda:aerial-combat-swarms/autopilot_bridge.git
    check_fail "mavbridge git clone (ssh)"
  else
    git clone https://yoda:18080/aerial-combat-swarms/autopilot_bridge.git
    check_fail "mavbridge git clone (http)"
  fi
else
  cd autopilot_bridge
  git checkout .  # reset to state where we can update
  check_fail "mavbridge git checkout ."
  git fetch origin  # update local copy of repo
  check_fail "mavbridge git fetch"
  git checkout master  # bring updates into working space
  check_fail "mavbridge git checkout dev"
fi

# Build all workspace packages
cd ~/acs_ros_ws/
catkin_make
check_fail "catkin_make"

echo ""
echo "Congratulations, your system has been updated. Please check that the correct branches of mavlink, autonomy-payload, and autopilot_bridge have been checked out and installed as appropriate."
echo ""

