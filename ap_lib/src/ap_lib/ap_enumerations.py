#!/usr/bin/env python

# Contains enumerations and constants for use throughout the ACS Python architecture

# Enumeration for available payload controller types
NO_PAYLOAD_CTRL = 0
WP_SEQUENCE_CTRLR = 1
FOLLOW_CTRLR = 2
LANDING_SEQUENCE_CTRLR = 3

# Mapping between controller modes and strings for GUI use or debugging
CTL_MODES = { 0: 'Autopilot', \
              1: 'Wpt Sequencer', \
              2: 'Follower',
              3: 'Land Sequencer' }

CTL_MODE_VALUES = { 'Autopilot': 0, \
                    'Wpt Sequencer': 1, \
                    'Follower': 2, \
                    'Land Sequencer': 3 }

# Fixed (per convention) state-specific waypoints
TAKEOFF_WP = 1       # Airborne
SWARM_STANDBY_WP = 4 # Available for tasking
SWARM_EGRESS_WP = 5  # Leaving swarm for recovery
RACETRACK_WP = 7     # First racetrack waypoint


# Enumeration for available swarm behaviors
SWARM_STANDBY = 0         # No swarm behavior (set no payload control)
FIXED_FOLLOW = 1          # Canned follow positions based on side #
SWARM_SEQUENCE_LAND = 98  # Land in order (low-to-high UAV)
SWARM_EGRESS = 99         # Egress the swarm for recovery

# Mapping between swarm behaviors and strings for GUI use or debugging
SWARM_BHVRS = {  0: 'Standby', \
                 1: 'Fixed Follow', \
                98: 'Sequence Land', \
                99: 'Egress' }

SWARM_BHVR_VALUES = { 'Standby': 0, \
                      'Fixed Follow': 1, \
                      'Sequence Land': 98, \
                      'Egress': 99 }

# Enumeration for swarming states
PRE_FLIGHT = 0    # Powered on, going through pre-fllight checks
FLIGHT_READY = 1  # Awaiting launch
INGRESS = 2       # Airborne, waiting for handoff to swarm operator
SWARM_READY = 3   # Available for swarm behavior (standby loiter)
SWARM_ACTIVE = 4  # Executing an active swarm behavior
EGRESS = 5        # Transit to recovery staging (still required?)
LANDING = 6       # Flight crew has control for landing
ON_DECK = 7       # Aircraft has landed
POST_FLIGHT = 8   # Post landing checks (will probably not be seen)


# Mapping between swarm states and strings for GUI use or debugging
STATE_STRINGS = { PRE_FLIGHT: 'Preflight', \
                  FLIGHT_READY: 'Flight Ready', \
                  INGRESS: 'Ingress', \
                  SWARM_READY: 'Swarm Ready', \
                  SWARM_ACTIVE: 'Swarm Active', \
                  EGRESS: 'Egress', \
                  LANDING: 'Landing', \
                  ON_DECK: 'On Deck',
                  POST_FLIGHT: 'Post Flight' }

STATE_VALUES = { 'Preflight': 0, \
                 'Flight Ready': 1, \
                 'Ingress': 2, \
                 'Swarm Ready': 3, \
                 'Swarm Active': 4, \
                 'Egress': 5, \
                 'Landing': 6, \
                 'On Deck': 7, \
                 'Post Flight': 8 }

# Mapping between autopilot modes and strings for GUI use or debugging
MODE_STRINGS = { 0:  'RTL', \
                 1:  'MANUAL', \
                 2:  'FBWA', \
                 3:  'GUIDED', \
                 4:  'AUTO', \
                 5:  'FBWB', \
                 6:  'CIRCLE', \
                 15: 'UNMAPPED' }
#UNMAPPED = ACRO, LOITER, INITIALIZING, TRAINING, STABILIZE, CRUISE

# Enumeration for types of waypoints that we might need to test for
WP_TYPE_NORMAL = 16
WP_TYPE_LOITER = 17
WP_TYPE_TURNS = 18
WP_TYPE_TIME = 19
WP_TYPE_LAND = 21
WP_TYPE_TAKEOFF = 22
WP_TYPE_LOITER_TO_ALT = 31

