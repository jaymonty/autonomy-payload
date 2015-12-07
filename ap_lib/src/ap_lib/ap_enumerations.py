#!/usr/bin/env python

# Contains enumerations and constants for use throughout the ACS Python architecture

# Fixed (per convention) state-specific waypoints
TAKEOFF_WP = 1       # Airborne
INGRESS_LOITER_WP = 3 # Ingress loiter for time waypoint at constant 75m altitude
INGRESS_CYLINDER_WP = 4 # Ingress loiter to altitude waypoint at calculated alt slot
SWARM_STANDBY_WP = 5 # Available for tasking
SWARM_EGRESS_WP = 6  # Leaving swarm for recovery  NOTE:  Not used right now
RACETRACK_WP = 7     # First racetrack waypoint
LAND_A_WP = 19       # First WP in the east-to-west (RW27) landing sequence
LAND_B_WP = 26       # First WP in the west-to-east (RW10) landing sequence

# Default Values. These get updated on startup and again on flight ready
MIN_REL_ALT = 70.0  # Minimum relative altitude that a controller can order
MAX_REL_ALT = 500.0 # Maximum relative altitude that a controller can order
MAX_ABS_LAT = 60.0  # Maximum absolute value that is commandable to lat
MAX_ABS_LON = 180.0 # Maximum absolute value that is commandable to lon

BASE_REL_ALT = 100.0   # Base rel_alt for "stacked" behaviors
ALT_BLOCK_SIZE = 15.0  # Altitude block size for altitude-separated behaviors

# Enumeration for available swarm behaviors
SWARM_STANDBY = 0          # No swarm behavior (set no payload control)
SWARM_LINEAR_FORMATION = 1 # Straight line high-to-low formation
SWARM_SEARCH = 2           # Conduct a coordinated search of a specified area
GREEDY_SHOOTER = 3         # Track and shoot the closest red UAVs
ALTITUDE_SORT = 4          # Consensus algorithm to sort UAVs by rel_alt
LAZY_ALTITUDE_SORT = 5     # Lazy consensus algorithm to sort UAVs by rel_alt
SWARM_SEQUENCE_LAND = 98   # Land in order (low-to-high UAV)
SWARM_EGRESS = 99          # Egress the swarm for recovery

# Mapping between swarm behaviors and strings for GUI use or debugging
SWARM_BHVRS = {  SWARM_STANDBY:          'Standby', \
                 SWARM_LINEAR_FORMATION: 'Line Formation', \
                 SWARM_SEARCH:           'Swarm Search', \
                 GREEDY_SHOOTER:         'Greedy Shooter', \
                 ALTITUDE_SORT:          'Altitude Sort', \
                 LAZY_ALTITUDE_SORT:     'Lazy Alt Sort', \
                 SWARM_SEQUENCE_LAND:    'Sequence Land' }

SWARM_BHVR_VALUES = { 'Standby':        SWARM_STANDBY, \
                      'Line Formation': SWARM_LINEAR_FORMATION, \
                      'Swarm Search':   SWARM_SEARCH, \
                      'Greedy Shooter': GREEDY_SHOOTER, \
                      'Altitude Sort':  ALTITUDE_SORT, \
                      'Lazy Alt Sort':  LAZY_ALTITUDE_SORT, \
                      'Sequence Land':  SWARM_SEQUENCE_LAND }

# Enumeration for swarming states
PRE_FLIGHT = 0    # Powered on, going through pre-fllight checks
FLIGHT_READY = 1  # Awaiting launch
INGRESS = 2       # Airborne, waiting for handoff to swarm operator
SWARM_READY = 3   # Available for swarm behavior
LANDING = 4       # Flight crew has control for landing
ON_DECK = 5       # Aircraft has landed
POST_FLIGHT = 6   # Post landing checks (will probably not be seen)
AP_ERROR = 7      # Error state (probably due to wrong autopilot mode)
LAST_STATE = 15   # CANNOT EXCEED 15; network message field is a nibble

# Mapping between swarm states and strings for GUI use or debugging
STATE_STRINGS = { PRE_FLIGHT:   'Preflight', \
                  FLIGHT_READY: 'Flight Ready', \
                  INGRESS:      'Ingress', \
                  SWARM_READY:  'Swarm Ready', \
                  LANDING:      'Landing', \
                  ON_DECK:      'On Deck',
                  POST_FLIGHT:  'Post Flight',
                  AP_ERROR:     'State Error' }

STATE_VALUES = { 'Preflight':    PRE_FLIGHT, \
                 'Flight Ready': FLIGHT_READY, \
                 'Ingress':      INGRESS, \
                 'Swarm Ready':  SWARM_READY, \
                 'Landing':      LANDING, \
                 'On Deck':      ON_DECK, \
                 'Post Flight':  POST_FLIGHT,
                 'State Error':  AP_ERROR }

# Enumeration for autopilot modes
RTL = 0
MANUAL = 1
FBWA = 2
GUIDED = 3
AUTO = 4
FBWB = 5
CIRCLE = 6
LOITER = 7
INITIALIZING = 8
UNMAPPED = 15

# Mapping between autopilot modes and strings for GUI use or debugging
MODE_STRINGS = { RTL:      'RTL', \
                 MANUAL:   'MANUAL', \
                 FBWA:     'FBWA', \
                 GUIDED:   'GUIDED', \
                 AUTO:     'AUTO', \
                 FBWB:     'FBWB', \
                 CIRCLE:   'CIRCLE', \
                 LOITER:   'LOITER', \
                 INITIALIZING: 'INITIALIZING', \
                 UNMAPPED: 'UNMAPPED' }

MODE_VALUES = { 'RTL':      RTL, \
                'MANUAL':   MANUAL, \
                'FBWA':     FBWA, \
                'GUIDED':   GUIDED, \
                'AUTO':     AUTO, \
                'FBWB':     FBWB, \
                'CIRCLE':   CIRCLE, \
                'LOITER':   LOITER, \
                'INITIALIZING' : INITIALIZING, \
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
WP_TYPE_ENABLE_FENCE = 207

# Air-to-air targeting envelope parameters (pretty large envelope for now)
MAX_RANGE = 200.0
FOV_WIDTH = 40.0   # 40 degree horizontal cutout in front of UAV
FOV_HEIGHT = 80.0  # 80 degree vertical cutout in front of UAV



