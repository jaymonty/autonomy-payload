#!/bin/bash

#------------------------------------------------------------------------------
# Purpose: Set up companion computer, once Ubuntu is installed
#          Written for ODroid U3 running "standard" Ubuntu install
# Author: Mike Clement
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
# Helper functions

# Function to check for command failure and print useful error messages
function check_fail
{
if [ $? != 0 ]; then
  echo -e "\nERROR: $1\n"
  kill -INT $$
fi
}

#------------------------------------------------------------------------------
# Parition various pieces of the install
# Note: we try to make each function idempotent, so it can safely be
# run over and over again, and used as an upgrade script.

#####
# Perform initial configuration (generally only needed once)
#####
function install_initial
{

# Make it so this user can do passwordless sudo :)
sudo grep -x "$USER ALL=(ALL) NOPASSWD: ALL" /etc/sudoers > /dev/null
if [ $? != 0 ]; then
  sudo chmod u+w /etc/sudoers
  check_fail "chmod (sudo permissions)"
  echo "$USER ALL=(ALL) NOPASSWD: ALL" | sudo tee -a /etc/sudoers > /dev/null
  check_fail "echo (sudo permissions)"
  sudo chmod u-w /etc/sudoers
  check_fail "chmod (sudo permissions)"
fi

# Set the hostname
sudo hostname $AIRCRAFT_NAME
check_fail "set temporary hostname"
sudo sh -c "echo $AIRCRAFT_NAME > /etc/hostname"
check_fail "set persistent hostname"
sudo sed -ri "s/\s+localhost.*/\tlocalhost\t$AIRCRAFT_NAME/" /etc/hosts
check_fail "set loopback hostname"

# Fix things that make SSH logins slow
grep UseDNS /etc/ssh/sshd_config > /dev/null
if [ $? == 0 ]; then
  sudo sed -i s/UseDNS\ yes/UseDNS\ no/ /etc/ssh/sshd_config
else
  sudo sh -c 'echo "UseDNS no" >> /etc/ssh/sshd_config'
fi
grep UsePAM /etc/ssh/sshd_config > /dev/null
if [ $? == 0 ]; then
  sudo sed -i s/UsePAM\ yes/UsePAM\ no/ /etc/ssh/sshd_config
else
  sudo sh -c 'echo "UsePAM no" >> /etc/ssh/sshd_config'
fi

# Disable services we don't plan to use
for s in bluetooth cups lightdm ntp saned spamassassin speech-dispatcher whoopsie; do
  sudo service $s stop
  sudo update-rc.d -f $s remove
done

# Update locale settings
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

# Disable automatic apt tasks
sudo rm /etc/cron.daily/apt

# Add ROS repo
ls /etc/apt/sources.list.d/ros-latest.list &> /dev/null
if [ $? != 0 ]; then
  sudo sh -c 'wget http://packages.namniart.com/repos/namniart.key -O - | apt-key add -'
  check_fail "ROS apt-key"
  sudo sh -c 'echo "deb http://packages.namniart.com/repos/ros raring main" > /etc/apt/sources.list.d/ros-latest.list'
  check_fail "ROS apt source"
fi

# Regrettably, we need to disable Git's SSL cert check
git config --global http.sslVerify false
check_fail "git_config"

# Remove any old wireless device entries from udev
# (this will make our wifi device 'wlan0')
sudo sh -c "echo -n > /etc/udev/rules.d/70-persistent-net.rules"
check_fail "echo (udev rules)"

# Create network configuration
cat > interfaces.tmp <<EOF
auto lo
iface lo inet loopback

auto eth0
iface eth0 inet static
    address 192.168.0.1
    netmask 255.255.255.0

auto wlan0
iface wlan0 inet static
    address 192.168.2.$AIRCRAFT_ID
    netmask 255.255.255.0
    gateway 192.168.2.1
    dns_nameservers 172.20.20.11
#    dns_nameservers 8.8.8.8
    wireless-mode ad-hoc
    wireless-essid zephyr
    wireless-channel 6
    wireless-ap 00:11:22:33:44:55
EOF
check_fail "cat (interfaces config)"
sudo mv interfaces.tmp /etc/network/interfaces
check_fail "mv (interfaces config)"
sudo chown root:root /etc/network/interfaces
check_fail "chown (interfaces config)"

# Stand up interface as-is for this boot
WLAN_DEV=`sudo ifconfig -a | grep wlan | awk '{print $1}'`
sudo ifconfig $WLAN_DEV down
check_fail "ifconfig down (wlan config)"
sudo iwconfig $WLAN_DEV mode ad-hoc essid zephyr channel 6 ap 00:11:22:33:44:55
check_fail "iwconfig (wlan config)"
sudo ifconfig $WLAN_DEV inet 192.168.2.$AIRCRAFT_ID/24 up
check_fail "ifconfig up (wlan config)"
sudo route add default gw 192.168.2.1
check_fail "route (wlan config)"
# NOTE: adjust the below as necessary
grep 172.20.20.11 /etc/resolv.conf
if [ $? != 0 ]; then
  sudo sh -c 'echo "nameserver 172.20.20.11" >> /etc/resolv.conf'
  check_fail "dns (wlan config)"
fi

}

#####
# Do base software installation
#####
function install_base_software
{

# Update package lists
sudo apt-get update
check_fail "apt-get update"

# Remove unneeded packages
sudo apt-get --assume-yes remove flash-kernel

# Upgrade existing packages
sudo apt-get --assume-yes upgrade
check_fail "apt-get upgrade"

# Install ROS and other useful packages
sudo apt-get --assume-yes install \
git \
ros-hydro-ros-base \
ros-hydro-sensor-msgs \
iperf \
python-netifaces \
python-setuptools \
screen \
batctl
check_fail "apt-get install"

# Set up ROS
# (some of these might fail after the first successful run, don't worry about error checking)
sudo rosdep init
rosdep update
grep "/opt/ros/hydro/setup.bash" ~/.bashrc
if [ $? != 0 ]; then
  echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
fi
source /opt/ros/hydro/setup.bash

# Clean up deb files to free up some space
sudo apt-get clean

# Install kernel with batman-adv support
# NOTE: this is a work in progress; need update support!
KERNEL_DPKG="linux-image-3.8.13.28"
KERNEL_FILE="${KERNEL_DPKG}-10.00.batman_armhf.deb"
dpkg -l | grep $KERNEL_DPKG &> /dev/null
if [ $? != 0 ]; then
  cd ~
  wget --no-check-certificate https://yoda.ern.nps.edu/download/$KERNEL_FILE
  check_fail "kernel download"
  sudo dpkg -i $KERNEL_FILE
  check_fail "kernel install"
  rm $KERNEL_FILE
fi

}

#####
# Install payload software from repos
#####
function install_payload_software
{

# Install (py)mavlink library
cd ~
ls mavlink/ &> /dev/null
if [ $? != 0 ]; then
  git clone https://yoda.ern.nps.edu:18080/aerial-combat-swarms/mavlink.git
  check_fail "mavlink git clone"
  cd mavlink/pymavlink/
  git checkout dev  # The yoda branch we use for production
  check_fail "mavlink git checkout dev"
else
  cd mavlink/pymavlink/
  git fetch origin  # update local copy of remote repo
  check_fail "mavlink git fetch"
  git checkout .  # discard any local changes
  check_fail "mavlink git checkout ."
  git clean -df  # discard any untracked files
  check_fail "mavlink git clean"
  git checkout dev  # have the right branch checked out
  check_fail "mavlink git checkout dev"
  git reset --hard origin/dev  # bring in updates
  check_fail "mavlink git reset"
fi

# Set dialect so we don't compile the universe
env | grep "MAVLINK_DIALECT" &> /dev/null
if [ $? != 0 ]; then
  export MAVLINK_DIALECT=ardupilotmega
fi
grep "MAVLINK_DIALECT" ~/.bashrc &> /dev/null
if [ $? != 0 ]; then
  echo "export MAVLINK_DIALECT=ardupilotmega" >> ~/.bashrc
fi

rm -rf ~/.local/  # Clear out old build
check_fail "mavlink remove .local/"
python setup.py build install --user
check_fail "mavlink setup.py"

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
  git clone https://yoda.ern.nps.edu:18080/aerial-combat-swarms/autonomy-payload.git
  check_fail "payload git clone"
else
  cd autonomy-payload
  git fetch origin  # update local copy of remote repo
  check_fail "payload git fetch"
  git checkout .  # discard any local changes
  check_fail "payload git checkout ."
  git clean -df  # discard any untracked files
  check_fail "payload git clean"
  git checkout master  # have the right branch checked out
  check_fail "payload git checkout master"
  git reset --hard origin/master  # bring in updates
  check_fail "payload git reset"
fi

# Clone or update the autopilot_bridge repo
cd ~/acs_ros_ws/src/
ls autopilot_bridge/ &> /dev/null
if [ $? != 0 ]; then
  git clone https://yoda.ern.nps.edu:18080/aerial-combat-swarms/autopilot_bridge.git
  check_fail "mavbridge git clone"
else
  cd autopilot_bridge
  git fetch origin  # update local copy of remote repo
  check_fail "mavbridge git fetch"
  git checkout .  # discard any local changes
  check_fail "mavbridge git checkout ."
  git clean -df  # discard any untracked files
  check_fail "mavbridge git clean"
  git checkout master  # have the right branch checked out
  check_fail "mavbridge git checkout master"
  git reset --hard origin/master  # bring in updates
  check_fail "mavbridge git reset"
fi

# Build all workspace packages
# NOTE: This now also builds all shared libs in autonomy-payload
cd ~/acs_ros_ws/
catkin_make clean
check_fail "catkin_make clean"
catkin_make
check_fail "catkin_make"

}

#####
# Function to set up auto-start of payload software
#####
function install_autostart
{

# Set up automatic start-on-boot
# (highly unlikely to have errors, ignore check_fail())
sudo cp ~/acs_ros_ws/src/autonomy-payload/deploy/init.d-script /etc/init.d/autonomy-payload
sudo sed -i s/AIRCRAFT_ID/$AIRCRAFT_ID/g /etc/init.d/autonomy-payload
sudo sed -i s/AIRCRAFT_NAME/$AIRCRAFT_NAME/g /etc/init.d/autonomy-payload
sudo sed -i s/USER/$USER/g /etc/init.d/autonomy-payload
sudo chown root:root /etc/init.d/autonomy-payload
sudo chmod 755 /etc/init.d/autonomy-payload
sudo update-rc.d autonomy-payload defaults

}

#------------------------------------------------------------------------------
# Main program flow

INSTALL_INIT=true
INSTALL_BASE=true

# Parse any arguments
while getopts ":uq" opt; do
  case $opt in
    u)
      INSTALL_INIT=false
      ;;
    q)
      INSTALL_INIT=false
      INSTALL_BASE=false
      ;;
    \?)
      echo ""
      echo "usage: $0 [-u] [-q]"
      echo "  -u   Update only (skip initial config)"
      echo "  -q   Quick update only (skip initial config AND base software install)"
      echo ""
      exit 0
      ;;
  esac
done

# We want to run as the actual user, not as root
if [ `whoami` == "root" ]; then
  echo "Please run as the user that will run the payload software."
  exit 1
fi

if [ $INSTALL_INIT == true ]; then
  # Collect any needed user information
  echo ""
  read -p "Please enter a unique numeric ID for this aircraft: " AIRCRAFT_ID
  if [ -z $AIRCRAFT_ID ]; then
    echo "No ID specified; aborting"
    exit 1
  fi
  read -p "Please enter a hostname for this aircraft (max 16 chars): " AIRCRAFT_NAME
  if [ -z $AIRCRAFT_NAME ]; then AIRCRAFT_NAME='odroid'; fi

  install_initial
fi

# Make sure user has Internet connection set up
echo ""
read -p "Please make sure you have an appropriate Internet gateway set up, then press Enter. "

# Stop any running payload
sudo service autonomy-payload stop

# Shut down NTP service and get a proper time hack
# (important for make to work correctly)
sudo service ntp stop
sudo ntpdate time.nps.edu  # This should be customized later
check_fail "ntp time sync"

# Conditionally perform base software install
if [ $INSTALL_BASE == true ]; then
  install_base_software
fi

# Always perform payload software install
install_payload_software

# If performing an initial install, set up autostart
if [ $INSTALL_INIT == true ]; then
  install_autostart
fi

# Final message(s) to user
echo ""
echo "Congratulations, your system has been updated."
echo "Please check that the correct branches of mavlink, autonomy-payload,"
echo "and autopilot_bridge have been checked out."
echo "Then, reboot to automatically start the payload software."
echo ""
ls ~/bags/*bag* &> /dev/null
if [ $? == 0 ]; then
  echo "WARNING: There are ROS bags in ~/bags/;"
  echo "you may wish to clear these off before flight!"
  echo ""
fi

