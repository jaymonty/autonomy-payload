#!/bin/bash

# Purpose: Set up a virtual bridge very specifically for ACS
# Author: Mike Clement

### Configuration and Functions ###

# Import some common config
LIB_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source $LIB_DIR/launch_lib.sh



### Do the setup ###

setup_bridge
