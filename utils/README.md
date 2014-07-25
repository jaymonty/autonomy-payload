# Various utilities

## net\_parser.py

Prints received ACS network messages to the screen.

Example (use 'lo' for SITL):

	python net_parser.py --device wlan0

Use '--help' for all options. Note that '--repeat' can be used to repeat all received messages but with source ID = 255 (0xff).

## latency.py

Super simple script that can help show messaging latency for bagged and possibly live messagesi (after minor modification). See the code for details.

## slave\_setup.py

Opens (or closes) a mavlink slave channel via the payload. Right now, slave channels should be UDP (TCP doesn't seem to work for providing a server, and there are no additional serial channels onboard).

Example (from a groundstation at 192.168.2.123 on interface wlan0 running e.g. MAVProxy on UDP port 1234, open a UDP slave channel on aircraft 5 (192.168.2.5)):

	python slave_setup.py --device wlan0 --target 5 --enable udp:192.168.2.123:1234

Use '--help' for all options. Use '--enable CHANNEL' to open a channel, and '--disable CHANNEL' to disable it.

## autopilot\_demo.sh

Can be used to test autopilot functionality via ROS message publications. NOTE: this is likely out of date, and should instead be used as examples for understanding the ROS topics and messages, rather than run as a script.

