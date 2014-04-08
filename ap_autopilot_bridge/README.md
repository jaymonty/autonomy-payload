# ap_autopilot_bridge : ROS package for bridging with autopilots

Currently only supports mavlink-based autopilots

## First time use

From the base workspace folder, run

	catkin_make

You will need mavlink (and pymavlink) installed. You can grab the stock version from here: https://github.com/mavlink/mavlink

You will also need a serial link to the aircraft, such as a UART, USB-TTL converter, or telemetry radio.

To run the bridge, first start roscore, then run

	rosrun ap_autopilot_bridge mavlink.py

The following options can be set after "...bridge.py":
	--device=/dev/your-serial-device
	--baudrate=<baudrate>
	--mavlink=/your/path/to/mavlink

