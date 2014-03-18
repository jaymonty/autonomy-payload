# ap_mavlink_bridge : ROS package for bridging with mavlink

## First time use

From the base workspace folder, run

	catkin_make

You will need mavlink (and pymavlink) installed. You can grab the stock version from here: https://github.com/mavlink/mavlink

You will also need a serial link to the aircraft, such as a UART, USB-TTL converter, or telemetry radio.

To run the bridge, first start roscore, then run

	rosrun ap_mavlink_bridge bridge.py

If your serial connection is /dev/ttyUSB0 and your mavlink folder is /home/$USER/virtPlane/mavlink (where $USER is your username), the above *should* work. Otherwise, you can use the following command line arguments:

--device=/dev/your-serial-device

--mavlink=/your/path/to/mavlink

