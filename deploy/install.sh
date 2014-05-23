#!/bin/bash

#------------------------------------------------------------------------------
# Purpose: Set up companion computer, once Ubuntu is installed
# Author: Mike Clement
#------------------------------------------------------------------------------

# We want to run as the actual user, not as root
if [ `whoami` == "root" ]; then
  echo "Please run as the user that will run the payload software."
  exit 1
fi

# Collect any needed user information
read -p "Please enter a unique numeric ID for this aircraft: " AIRCRAFT_ID

#------------------------------------------------------------------------------
# General setup

# Make it so this user can do passwordless sudo :)
sudo grep -x "$USER ALL=(ALL) NOPASSWD: ALL" /etc/sudoers > /dev/null
if [ $? != 0 ]; then
  sudo chmod u+w /etc/sudoers
  echo "$USER ALL=(ALL) NOPASSWD: ALL" | sudo tee -a /etc/sudoers > /dev/null
  sudo chmod u-w /etc/sudoers
fi

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

# Disable NTP, user will set date/time
# (Used by autonomy-payload to set time from GPS)
#sudo chmod u+s /bin/date
sudo update-rc.d ntp disable
sudo service ntp stop

# Regrettably, we need to disable Git's SSL cert check
git config --global http.sslVerify false

# Remove any old wireless device entries from udev
# (this will make our wifi device 'wlan0')
sudo sh -c "echo -n > /etc/udev/rules.d/70-persistent-net.rules"

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
sudo mv interfaces.tmp /etc/network/interfaces
sudo chown root:root /etc/network/interfaces

# Stand up interface as-is for this boot
WLAN_DEV=`sudo ifconfig -a | grep wlan | awk '{print $1}'`
sudo ifconfig $WLAN_DEV down
sudo iwconfig $WLAN_DEV mode ad-hoc essid zephyr channel 6 ap 00:11:22:33:44:55
sudo ifconfig $WLAN_DEV inet 192.168.2.$AIRCRAFT_ID/24 up
sudo route add default gw 192.168.2.1
grep 172.20.20.11 /etc/resolv.conf
if [ $? != 0 ]; then
  sudo sh -c 'echo "nameserver 172.20.20.11" >> /etc/resolv.conf'
fi

# Make sure user has Internet connection set up
read -p "Please make sure you have an appropriate Internet gateway set up, then press Enter. "

#------------------------------------------------------------------------------
# Install packages 

# Update locale settings
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

# Add ROS repo
ls /etc/apt/sources.list.d/ros-latest.list &> /dev/null
if [ $? != 0 ]; then
  sudo sh -c 'echo "deb http://packages.namniart.com/repos/ros raring main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo wget http://packages.namniart.com/repos/namniart.key -O - | apt-key add -
fi

# Update package lists
sudo apt-get update

# Upgrade existing packages
sudo apt-get --assume-yes upgrade

# Install ROS and other useful packages
sudo apt-get --assume-yes install \
ros-hydro-ros-base \
ros-hydro-sensor-msgs \
ros-hydro-robot-pose-ekf \
iperf \
python-setuptools \
screen \
batctl

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

cd ~
ls mavlink/ &> /dev/null
if [ $? != 0 ]; then
  git clone https://github.com/mavlink/mavlink.git
  cd mavlink/pymavlink/
else
  cd mavlink/pymavlink/
  git pull
fi

python setup.py build install --user

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
  git clone https://yoda.ern.nps.edu:18080/aerial-combat-swarms/autonomy-payload.git
else
  cd autonomy-payload
  git pull
fi

# Build the autonomy-payload
cd ~/acs_ros_ws/
catkin_make

# Set up automatic start-on-boot
sudo cp ~/acs_ros_ws/src/autonomy-payload/deploy/init.d-script /etc/init.d/autonomy-payload
sudo sed -i s/AIRCRAFT_ID/$AIRCRAFT_ID/g /etc/init.d/autonomy-payload
sudo sed -i s/USER/$USER/g /etc/init.d/autonomy-payload
sudo chown root:root /etc/init.d/autonomy-payload
sudo chmod 755 /etc/init.d/autonomy-payload
sudo update-rc.d autonomy-payload defaults

#------------------------------------------------------------------------------
# Install BATMAN-adv module
# THIS IS A WORK IN PROGRESS, HARDKERNEL DOES WEIRD KERNEL STUFF

# Either clone the repo, or make sure it's updated
#ls batman-adv/ &> /dev/null
#if [ $? != 0 ]; then
#  git clone http://git.open-mesh.org/batman-adv.git batman-adv
#  cd batman-adv/
#else
#  cd batman-adv/
#  git pull
#fi

# Build and install the module
#make && make install

#------------------------------------------------------------------------------
# Clean-up

sudo apt-get clean

echo ""
echo "Congratulations, your system should be ready to use. Please reboot before running payload software, to make sure all settings have been applied."
echo ""

