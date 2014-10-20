#!/bin/bash

read -p 'Please make sure SITL and payload are running...'

read -p 'Press Enter to arm the throttle'
rostopic pub -1 /autopilot/arm std_msgs/Bool True

read -p 'Press Enter to go to AUTO mode'
rostopic pub -1 /autopilot/mode std_msgs/UInt8 3

read -p 'Press Enter to go to Waypoint 3'
rostopic pub -1 /autopilot/waypoint_goto std_msgs/UInt16 3

read -p 'Press Enter to go to a GUIDED point'
rostopic pub -1 /autopilot/guided_goto autopilot_bridge/LLA '{lat: 35.719597, lon: -120.763281, alt: 150}'

read -p 'Press Enter to return to racetrack in AUTO mode'
rostopic pub -1 /autopilot/mode std_msgs/UInt8 -- 3

read -p 'Press Enter to go to Rally point'
rostopic pub -1 /autopilot/mode std_msgs/UInt8 -- 0

read -p 'Press Enter to initiate landing'
rostopic pub -1 /autopilot/land std_msgs/Empty

echo "Demo complete."

