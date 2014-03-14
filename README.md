# autonomy-payload : Software for the onboard controller payload

A collection of ROS nodes for onboard navigation, safety monitoring, etc, and interfaces to the autopilot and other networked devices.

## Setting up on a development computer

These are the rough steps needed to clone the repo on a computer that already has ROS Groovy installed and configured. Eventually, these steps will be in a script, probably in APM-PX4 scripts.

	mkdir -p ~/acs_ros_ws/src/
	cd ~/acs_ros_ws/src/
	catkin_init_workspace
	wstool init		# unsure what this is doing
	cd ~/acs_ros_ws/
	catkin_make
	echo "source ~/acs_ros_ws/devel/setup.bash" >> ~/.bashrc
	source ~/acs_ros_ws/devel/setup.bash
	cd ~/acs_ros_ws/src/
	git clone git@yoda:aerial-combat-swarms/autonomy-payload.git

