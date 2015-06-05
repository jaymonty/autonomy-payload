#!/usr/bin/env python

# Contains enumerations and constants for use throughout the ACS Python architecture

# Enumeration for available payload controller types
NO_PAYLOAD_CTRL = 0
WP_SEQUENCE_CTRLR = 1
FOLLOW_CTRLR = 2
LANDING_SEQUENCE_CTRLR = 3

# Mapping between controller modes and strings for GUI use or debugging
CTL_MODES = { NO_PAYLOAD_CTRL:        'Autopilot', \
              WP_SEQUENCE_CTRLR:      'Wpt Sequencer', \
              FOLLOW_CTRLR:           'Follower',
              LANDING_SEQUENCE_CTRLR: 'Land Sequencer' }

CTL_MODE_VALUES = { 'Autopilot':      NO_PAYLOAD_CTRL, \
                    'Wpt Sequencer':  WP_SEQUENCE_CTRLR, \
                    'Follower':       FOLLOW_CTRLR, \
                    'Land Sequencer': LANDING_SEQUENCE_CTRLR }

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
SWARM_BHVRS = {  SWARM_STANDBY:       'Standby', \
                 FIXED_FOLLOW:        'Fixed Follow', \
                 SWARM_SEQUENCE_LAND: 'Sequence Land', \
                 SWARM_EGRESS:        'Egress' }

SWARM_BHVR_VALUES = { 'Standby':       SWARM_STANDBY, \
                      'Fixed Follow':  FIXED_FOLLOW, \
                      'Sequence Land': SWARM_SEQUENCE_LAND, \
                      'Egress':        SWARM_EGRESS }

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
STATE_STRINGS = { PRE_FLIGHT:   'Preflight', \
                  FLIGHT_READY: 'Flight Ready', \
                  INGRESS:      'Ingress', \
                  SWARM_READY:  'Swarm Ready', \
                  SWARM_ACTIVE: 'Swarm Active', \
                  EGRESS:       'Egress', \
                  LANDING:      'Landing', \
                  ON_DECK:      'On Deck',
                  POST_FLIGHT:  'Post Flight' }

STATE_VALUES = { 'Preflight':    PRE_FLIGHT, \
                 'Flight Ready': FLIGHT_READY, \
                 'Ingress':      INGRESS, \
                 'Swarm Ready':  SWARM_READY, \
                 'Swarm Active': SWARM_ACTIVE, \
                 'Egress':       EGRESS, \
                 'Landing':      LANDING, \
                 'On Deck':      ON_DECK, \
                 'Post Flight':  POST_FLIGHT }

# Enumeration for autopilot modes
RTL = 0
MANUAL = 1
FBWA = 2
GUIDED = 3
AUTO = 4
FBWB = 5
CIRCLE = 6
UNMAPPED = 15

# Mapping between autopilot modes and strings for GUI use or debugging
MODE_STRINGS = { RTL:      'RTL', \
                 MANUAL:   'MANUAL', \
                 FBWA:     'FBWA', \
                 GUIDED:   'GUIDED', \
                 AUTO:     'AUTO', \
                 FBWB:     'FBWB', \
                 CIRCLE:   'CIRCLE', \
                 UNMAPPED: 'UNMAPPED' }

MODE_VALUES = { 'RTL':      RTL, \
                'MANUAL':   MANUAL, \
                'FBWA':     FBWA, \
                'GUIDED':   GUIDED, \
                'AUTO':     AUTO, \
                'FBWB':     FBWB, \
                'CIRCLE':   CIRCLE, \
                'UNMAPPED': UNMAPPED }
#UNMAPPED = ACRO, LOITER, INITIALIZING, TRAINING, STABILIZE, CRUISE

# Enumeration for types of waypoints that we might need to test for
WP_TYPE_NORMAL = 16
WP_TYPE_LOITER = 17
WP_TYPE_TURNS = 18
WP_TYPE_TIME = 19
WP_TYPE_LAND = 21
WP_TYPE_TAKEOFF = 22
WP_TYPE_LOITER_TO_ALT = 31
WP_TYPE_LAND_SEQUENCE = 189

