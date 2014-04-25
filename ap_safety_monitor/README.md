# ap_safety_monitor : ROS package for monitoring safety and aircraft health

Publishes "heartbeat" to /safety/heartbeat iff all is well. Note: Heartbeat.msg has room for a counter, which is currently unused.

Can enable/disable by setting 1/0 to service /safety/set_health:

	rosservice call /safety/set_health "enable: 1"
