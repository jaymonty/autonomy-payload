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

