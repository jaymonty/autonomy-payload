#!/bin/bash

# Last-minute updates and fixes

# Function to check for command failure and print useful error messages
function check_fail
{
if [ $? != 0 ]; then
  echo -e "\nERROR: $1\n"
  kill -INT $$
fi
}

# Set correct timezone
grep 'US/Pacific' /etc/timezone > /dev/null
if [ $? != 0 ]; then
  sudo sh -c "echo 'US/Pacific' > /etc/timezone"
  check_fail "timezone echo"
  sudo dpkg-reconfigure -f noninteractive tzdata
  check_fail "timezone update"
fi

# Remove any old wireless device entries from udev, and add a
# generic rule to make *any* wifi device 'wlan0'
# NOTE: replicated in initial config stage
cat > udev.tmp <<EOF
# Catch-all for wlan devices
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="wlan*", NAME="wlan0"
EOF
check_fail "cat (udev rules)"
sudo mv udev.tmp /etc/udev/rules.d/70-persistent-net.rules
check_fail "mv (udev rules)"
sudo chown root:root /etc/udev/rules.d/70-persistent-net.rules
check_fail "chown (udev rules)"

