#!/bin/bash

#------------------------------------------------------------------------------
# Purpose: Set up development machine running "standard" Ubuntu install
# Author: Mike Clement
#------------------------------------------------------------------------------

# We want to run as the actual user, not as root
if [ `whoami` == "root" ]; then
  echo "Please run as the user that will run the payload software."
  exit 1
fi

read -p "Do you want to want to install (py)mavlink? (y/N) " INSTALL_MAVLINK

#------------------------------------------------------------------------------
# General setup

# Regrettably, we need to disable Git's SSL cert check
#git config --global http.sslVerify false

#------------------------------------------------------------------------------
# Install packages 

# Update locale settings
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

# Add ROS repo
ls /etc/apt/sources.list.d/ros-latest.list &> /dev/null
if [ $? != 0 ]; then
  sudo sh -c 'echo "deb http://packages.namniart.com/repos/ros raring main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo sh -c 'wget http://packages.namniart.com/repos/namniart.key -O - | apt-key add -'
fi

# Update package lists
sudo apt-get update

# Upgrade existing packages
sudo apt-get --assume-yes upgrade

# Install ROS and other useful packages
sudo apt-get --assume-yes install \
ros-hydro-desktop-full \
ros-hydro-sensor-msgs \
ros-hydro-robot-pose-ekf \
python-setuptools

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

if [[ $INSTALL_MAVLINK == "y" ]]; then
  cd ~

  ls mavlink/ &> /dev/null
  if [ $? != 0 ]; then
    git clone https://github.com/mavlink/mavlink.git
    cd mavlink/pymavlink/
    git remote add yoda https://yoda.ern.nps.edu:18080/aerial-combat-swarms/mavlink.git
    git fetch yoda
  else
    cd mavlink/pymavlink/
    git fetch origin
    git fetch yoda
  fi

  python setup.py build install --user
fi

#------------------------------------------------------------------------------
# Set up autonomy-payload

# Initialize the ROS workspace
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
  git clone git@yoda:aerial-combat-swarms/autonomy-payload.git
  if [ $? != 0 ]; then
    echo -e "\nPlease check your SSH keys in GitLab and try again."
    exit 1
  fi
else
  cd autonomy-payload
  git fetch origin
fi

# Install ACS shared libs
cd ~/acs_ros_ws/src/autonomy-payload/lib/
python setup.py build install --user

# Clone or update the autopilot_bridge repo
cd ~/acs_ros_ws/src/
ls autopilot_bridge/ &> /dev/null
if [ $? != 0 ]; then
  git clone git@yoda:aerial-combat-swarms/autopilot_bridge.git
  if [ $? != 0 ]; then
    echo -e "\nPlease check your SSH keys in GitLab and try again."
    exit 1
  fi
else
  cd autopilot_bridge
  git fetch origin
fi

# Build all workspace packages
cd ~/acs_ros_ws/
catkin_make

echo ""
echo "Congratulations, your system has been updated. Please check that the correct branches of mavlink, autonomy-payload, and autopilot_bridge have been checked out and installed as appropriate."
echo ""

