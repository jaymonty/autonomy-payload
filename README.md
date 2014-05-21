# autonomy-payload : a ROS-based stack for cooperative aerial robot autonomy

A collection of ROS nodes for perception, path planning, safety monitoring, etc, and interfaces to the autopilot and other networked aircraft and ground control stations. Originally designed to be run from a companion computer (e.g., RPi or ODroid) onboard a UAV using an ArduPlane-based autopilot (e.g., APM or PX4).

## Requirements

This is currently being developed against ROS Hydro on an Ubuntu-esque system. For best compatibility, use Ubuntu 12.04 LTS or newer (Xubuntu, etc should be fine as well).

You will need the following ROS packages for your distribution:

	sudo apt-get install ros-hydro-ros-base ros-hydro-sensor-msgs ros-hydro-robot-pose-ekf

See your distribution's ROS documentation for install instructions. The short version, which generally works:

	sudo rosdep init
	rosdep update
	echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
	source /opt/ros/hydro/setup.bash

In addition, to interface with MAVLink-based autopilots, you will need:

	sudo apt-get install python-setuptools

To install MAVLink for just your user (recommended):

	cd
	git clone https://github.com/mavlink/mavlink.git
	cd mavlink/pymavlink/
	python setup.py build install --user

Optionally, you can set permissions for normal users to update the date and time, and disable NTP, so the companion computer time is set from autopilot GPS when the autopilot bridge node starts. This is a **good thing** if you plan to use the EKF state estimator:

	sudo chmod u+s /bin/date
	sudo update-rc.d ntp disable
	sudo service ntp stop

## Installation

The steps we use for installation are below. Adjust the workspace directory name, etc, as you see fit.

	mkdir -p ~/acs_ros_ws/src/
	cd ~/acs_ros_ws/src/
	catkin_init_workspace
	cd ~/acs_ros_ws/
	catkin_make
	echo "source ~/acs_ros_ws/devel/setup.bash" >> ~/.bashrc
	source ~/acs_ros_ws/devel/setup.bash
	cd ~/acs_ros_ws/src/
	git clone # Path to repository
	cd ~/acs_ros_ws/
	catkin_make

To run, edit autonomy-payload/ap_master/launch/master.launch as needed, then run:

	roslaunch ap_master master.launch id:=AIRCRAFT_ID

replacing AIRCRAFT_ID with an aircraft-unique numeric ID (right now, constrained to 16-bit unsigned ints, so roughly 1..65535).

