# Various utilities

## flight\_tech.py

Discovers online aircraft, displays in a color-coded list, and provides mechanism for establishing a slave MAVProxy connection through the payload to perform pre-flight.

NOTE: VERY MUCH a work in progress

Color coding:
Red = Discovered but currently not broadcasting ("offline")
Yellow = Discovered and online, but "not ready" (not pre-flighted)
Green = Discovered, online, and ready for flight
Gray = In-flight, some features are inaccessible

Example (assuming computer's wlan0 is on 192.168.2.x network):

	python flight_tech.py --device wlan0

## ground\_heartbeat.py

Periodically broadcasts a heartbeat, consumed by the payload and used to indicate nominal link conditions to the autopilot.

Example:

	python ground_heartbeat.py --device wlan0

## launch\_payload.sh

Launches an instance of the payload in one of two ways:

* using a calculated offset for the network bridge's UDP port. In this case, communication between multiple payload instances on a single computer relies on a UDP relay, such as repeater.py (see below).

* using a network namespace container. Creates a virtual interface pair, gives one end a 192.168.2.x/24 address inside the container, and attaches the other end to a software bridge (sitl\_bridge). If the bridge doesn't exist, it will be created. Finally, it launches sitl.launch with reasonable parameters.

This script takes one numeric positional argument that should equal the "-I" (instance) argument to sim\_vehicle.sh. It further takes optional argument -C to run using containers and -R to specify the user to run the payload software as (only applicable when using -C).

Example usage with optional user specification:

	(terminal 1): sim_vehicle.sh -I 2 -v ArduPlane -L McMillan --aircraft testFolder
	(terminal 2): ./launch_payload.sh -C -R acsuser 2

## net\_parser.py

Prints received ACS network messages to the screen.

Example (use 'lo' for SITL):

	python net_parser.py --device wlan0

Use '--help' for all options. Note that '--repeat' can be used to repeat all received messages but with source ID = 255 (0xff).

## repeater.py

This, plus sitl\_ns.launch and some updates to the network bridge, offers an alternative to launch\_payload\_container.sh without needing containers (or root access). repeater.py can be configured to listen on loopback IP 127.0.1.1 at several UDP ports to traffic from those same port numbers at IP 127.0.0.1. When it hears a datagram from one of those ports, it repeats to all other ports. Thus, multiple payload instances can run at _different UDP ports on the same address_ and this utility will serve the same purpose as a broadcast address.

Example usage:

First, start repeater.py for however many payloads you wish to run:

	python repeater.py 2

Next, start SITLs as before with different -I values.

Then, start corresponding payloads using roslaunch. These options are approximately what launch\_payload\_container.sh provides, plus a new option ns:=NAME, where name is the ROS group or "namespace" (not to be confused with Linux network namespaces):

	roslaunch ap_master sitl_ns.launch id:=1 name:=sitl1 sitl:=tcp:127.0.0.1:5772 port:=5554 ns:=sitl1

Do the same for the second payload instance, adjusting all parameters (note that repeater.py by default listens on 5554 and consecutively higher numbers):

	roslaunch ap_master sitl_ns.launch id:=2 name:=sitl2 sitl:=tcp:127.0.0.1:5782 port:=5555 ns:=sitl2

If you wish to run utilities such as net\_parser.py or flight\_tech.py, you will need to request an extra repeated port from repeater.py:

	python repeater.py -e 1234 2

and use the --port and --lo-reverse options in those utilities:

	python net_parser.py --device lo --lo-reverse --port 1234

## slave\_setup.py

Opens (or closes) a mavlink slave channel via the payload. Right now, slave channels should be UDP (TCP doesn't seem to work for providing a server, and there are no additional serial channels onboard).

Example (from a groundstation at 192.168.2.123 on interface wlan0 running e.g. MAVProxy on UDP port 1234, open a UDP slave channel on aircraft 5 (192.168.2.5)):

	python slave_setup.py --device wlan0 --target 5 --enable udp:192.168.2.123:1234

Use '--help' for all options. Use '--enable CHANNEL' to open a channel, and '--disable CHANNEL' to disable it.

## visualizer.py

Simple visualization for the swarm, run within the ROS environment of a payload. See the code for details.

# Other utilities

These are various scripts that might come in handy but that require some hand-editing for use or that must be used with caution.

## autopilot\_demo.sh

Can be used to test autopilot functionality via ROS message publications. NOTE: this is likely out of date, and should instead be used as examples for understanding the ROS topics and messages, rather than run as a script.

## latency.py

Super simple script that can help show messaging latency for bagged and possibly live messagesi (after minor modification). See the code for details.

