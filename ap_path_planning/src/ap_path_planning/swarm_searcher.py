#!/usr/bin/env python

# Swarm_searcher code implemented as a node.
#
# Dylan Lau (Swarm_searcher impementation as object) 2015
# Modified by Duane Davis (object implementation) Aug 2015
# Updated to work with new ACS controller architecture Sep 2015
# NOTE: Conversion only--code needs a lot of cleanup to incorporate cleanly

# Standard python library imports
import sys
import time
import math
from argparse import ArgumentParser

# ROS library imports
import rospy
import std_msgs.msg as stdmsg

# ACS imports
import ap_msgs.msg as apmsg
import ap_srvs.srv as apsrv
import ap_lib.ap_enumerations as enums
import ap_lib.bitmapped_bytes as bytes
import ap_lib.nodeable as nodeable
import ap_lib.waypoint_behavior as wp_behavior
import autopilot_bridge.srv as apbrgsrv
import autopilot_bridge.msg as apbrgmsg
from ap_lib.gps_utils import *

# Base name for node topics and services
NODENAME = 'swarm_searcher'

# Global variables (constants)
# Local Enumeration for swarming uav search states
SEARCH_READY = 1   # Awaiting search area
SEARCH_INGRESS = 2 # Received search area and transiting to it
SEARCH_ACTIVE = 3  # Searching in search area
SEARCH_EGRESS = 4  # search end/completed and transiting to staging area
SEARCH_TRACKING = 5# search uav detected something is tasked to track it
SEARCH_FAULT = 99  # search uav has shown a fault and kept out of the operation

SEARCH_CELL_FULLY_VISITED = 0 # All cells visited
SEARCH_CELL_FULLY_ASSIGNED = 1 # All cells assigned
SEARCH_CELL_STILL_AVAILABLE = 2 # Cells still available for assignment

SEARCH_CELL_RADIUS = 150 # Interval between search cell in metres
SEARCH_ALTITUDE = 50 # Height to conduct search in metres
SEARCH_SAFETY_ALTITUDE_INTERVAL = 15 # Height between UAV of the same cell in metres
SEARCH_CELL_THRESHOLD = 100 # Distance to determine that cell is searched (must be greater than infinite loiter radius
SEARCH_ARRIVAL_THRESHOLD = 150 # Distance to start assigning cells in search grid to UAV
SEARCH_MAX_THRESHOLD = 100000000 # Max number for comparsion
SEARCH_TIMEOUT_INTERVAL = rospy.Duration(15) # 15 secs interval to check if searcher get closer to assigned waypoint
SEARCH_TIMEOUT_STRIKES = 3 # Number of strikes a UAV can get from timeout before being thrown out of the search operation
SEARCH_TIMEOUT_LOITERDISTANCEBUFFER = 10 # Buffer for UAV doing loiter such as Egressing after reaching the swarming waypoint

SEARCH_SWARM_SEARCH_READY_STATE = enums.SWARM_READY # enum state when UAV is ready for swarm behavior
SEARCH_SWARM_SEARCH_ACTIVE_STATE = enums.SWARM_READY # enum state when UAV is executing an active swarm behavior


# "struct" to store data for search search cell
class cell():
    LLA=None
    grid=None
    assigned=False
    assignedTo=None
    visited=False
    visitedBy=None
    visitedTimeStamp=None

# "struct" to info of each search UAV
class searchUAVdata():
    status=None
    subSwarmID=None
    receivedState=None
    pose=None
    assignedAltitude=None
    assignedCell=None
    assignedTime=None
    assignedLLA=None
    preplannedPath=0
    originalLLA=None
    timeoutdistance=None
    timeoutTime=None
    timeoutCounter=0


# Class member variables:
#
# Inherited from Nodeable:
#   nodeName:  Name of the node to start or node in which the object is
#   timer: ROS rate object that controls the timing loop
#   DBUG_PRINT: set true to force screen debug messages (default FALSE)
#   WARN_PRINT: set false to force screen warning messages (default FALSE)
#
# Class member functions:
class SwarmSearcher(wp_behavior.WaypointBehavior):

    search_master_searcher_id = None #variable for master searcher id
    #2D Array to store the 'cell' objects
    searchGrid = None
    # Search Swarm UAV status
    searchUAVMap = dict()

    # Assume Search Grid message states BottomLeft LLA plus length and width
    #bottomLeftLLA = [35.718788,-120.768071]
    bottomLeftLLA = None

    centerofSearchGridLLA = None

    # assume length and width divided by SEARCH_CELL_RADIUS (150 metres) equals cols and rows
    # may need to reduce to keep bigger buffer btw fence and search area
    #cols = 5
    cols = None
    #rows = 4
    rows = None

    # default search Algo enum
    #selectedSearchAlgo = 1
    selectedSearchAlgo = None
    numOfSearcher = 0
    rowSearch = False
    prePlannedPathPlanned = 0


    # Class initializer initializes class variables.
    # This assumes that the object is already running within an initialized
    # ROS node (i.e., the object does not initialize itself as a node).
    # This enables multiple objects to run within a single node if
    # required.  The initializer does some parameter checking (mostly
    # whether or not all of the required parameters are there), but does
    # not check ranges, magnitudes, or signs.
    # @param nodename: name of the ROS node in which this object exists
    # @param ownAC: ID (int) of this aircraft
    #def __init__(self, nodename, ownAC):
    #    nodeable.Nodeable.__init__(self, nodename)
    def __init__(self, nodename, ownAC, args):
        wp_behavior.WaypointBehavior.__init__(self, nodename, \
            enums.SWARM_SEARCH)
        self.DBUG_PRINT = False
        self.WARN_PRINT = False
        # Boolean flag to determine Centralised search node
        self.SEARCH_MASTER = False
        self.SEARCH_STATUS = SEARCH_READY
        self.ownID = ownAC
        self.ExecuteMasterSearcherBehavior = False

        # Used to determine altitude for wpt orders
        self._wp_rel_alt = None

        # Counter used to assign safety altitude to each UAV
        self.uavAltitudeCounter = 0

        # Time when search operation begins
        self.search_Operation_StartTime = None
        # Time when current search runs begins
        self.search_Run_StartTime = None
        # Boolean when first search cell found
        self.search_Run_firstSearch = False

        # Boolean flag to determine if any waypoint message to send
        self.wpmsgQueued = False

        self._getWpSrvProxy = None
        self._swarmSearchPublisher = None
        self._wps_to_send = bytes.SearchWaypointParser()

#        self.DBUG_PRINT = True
#        self.INFO_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    def publisherSetup(self):
        self._swarmSearchPublisher = \
            self.createPublisher("send_swarm_behavior_data", apmsg.BehaviorParameters, 1)


    def serviceProxySetup(self):
        self._getWpSrvProxy = \
            self.createServiceProxy("wp_getrange", apbrgsrv.WPGetRange)


    def set_behavior(self, params):
        ''' Sets behavior parameters based on set service parameters
        @param params: parameters from the set service request
        @return True if set with valid parameters
        '''
        self.subscribe_to_swarm()
        parser = bytes.SearchOrderParser()
        parser.unpack(params)

        try:
            if self._ap_wp is None:
                raise ValueError('Autopilot status has not been received yet')

            # Use the rel_alt of the infinite loiter waypoint for swarm search WP rel_alts
            self._last_wp_id = int(rospy.get_param("last_mission_wp_id"))
            self._wp_rel_alt = \
                self._getWpSrvProxy(self._last_wp_id, self._last_wp_id)
            if self._wp_rel_alt.ok:
                self._wp_rel_alt = self._wp_rel_alt.points[0].z
            else:
                raise Exception('Failed to fetch infinite loiter waypoint')

            if parser.masterID not in self._swarm:
                raise Exception('Attempt to initialize search without master (%d) in swarm'\
                                %parser.masterID)

            self.search_master_searcher_id = parser.masterID
            if self.ownID == self.search_master_searcher_id:
                self.log_info("Initializing swarm search as master")

                if self.ExecuteMasterSearcherBehavior == False:
                    self.search_Operation_StartTime = rospy.Time.now()
                    self.bottomLeftLLA = [parser.lat, parser.lon]
                    self.selectedSearchAlgo = 1 # set 1 (Greedy) as default
                    tempVar = parser.areaLength / SEARCH_CELL_RADIUS
                    self.cols = int(math.ceil(tempVar))
                    tempVar = parser.areaWidth / SEARCH_CELL_RADIUS
                    self.rows = int(math.ceil(tempVar))
                    self.ExecuteMasterSearcherBehavior = True
                    self.SEARCH_STATUS = SEARCH_READY
                    self.searchGrid = None
                    self.log_dbug("Initializing swarm search as master at time %s"\
                                  %self.search_Operation_StartTime)

                    # dtd: This ends up being ignored (I think)
                    for vehicle in self.searchUAVMap:
                        self.searchUAVMap[vehicle].assignedAltitude = self._wp_rel_alt
                        # dtd:  I added this line because it didn't work until I did
                        #       No idea where it was being done originally
                        self.searchUAVMap[vehicle].status = SEARCH_READY

                    self._activate_swarm_search(parser.algorithmNumber)
                    self.log_dbug("Using swarm search algorithm %d" %self.selectedSearchAlgo)
                    self.set_ready_state(True)

                else:
                    self.log_warn("Swarm search already underway--suspend operation before reinitializing")

            else:
                self.ExecuteMasterSearcherBehavior = False
                self.searchUAVMap[self.ownID].assignedAltitude = self._wp_rel_alt
                self.log_info("Swarm search initialized as slave with master %d and altitude %f"
                              %(parser.masterID, self.searchUAVMap[self.ownID].assignedAltitude))
                self.SEARCH_STATUS = SEARCH_READY
                self.set_ready_state(True)

            self.wpmsgQueued = False
            return True

        except Exception as ex:
            self.log_warn("Failed to initialize swarm search: " + str(ex))
            self.set_ready_state(False)
            return False


    def run_behavior(self):
        if self.ExecuteMasterSearcherBehavior == True:
            self.wpmsgQueued = False #flag to determine if any waypoint message to send

            for vehicle in self.searchUAVMap:
                self.log_dbug("TimeLoop: UAV [" + str(vehicle) + \
                              "] Received State [" + str(self.searchUAVMap[vehicle].receivedState) + \
                              "] pose: " + str(self.searchUAVMap[vehicle].pose))

            if self.SEARCH_STATUS == SEARCH_READY:
                self.SEARCH_STATUS = SEARCH_INGRESS
                self.search_Run_StartTime = rospy.Time.now()
                self.log_dbug("Swarm Search Master [" + str(self.ownID) + \
                              "] current search run started on " + \
                              str(rospy.Time.now()))
                self.search_Run_firstSearch = False
            elif self.SEARCH_STATUS == SEARCH_INGRESS:
                self._assign_ready_UAVs_to_searchGrid()
                self._check_timeout()
                # Check if any of the search uav has reached the search area. set search status to active if true
                reached = self._assign_ingress_UAVs_to_search()

                if reached == True:
                    self.SEARCH_STATUS = SEARCH_ACTIVE
            elif self.SEARCH_STATUS == SEARCH_ACTIVE:
                self._assign_ready_UAVs_to_searchGrid()
                self._assign_ingress_UAVs_to_search()
                self._check_timeout()

                # Check if search is completed (all cells visited) set status to SEARCH_EGRESS
                searchComplete = self._check_search_operation()

                if searchComplete == True:
                    currentTime = rospy.Time.now()
                    timeDelta = currentTime - self.search_Run_StartTime
                    self.log_dbug("Swarm Search Master [" + str(self.ownID) + \
                                  "] search completed after " + str(timeDelta/1e9) + " secs")

                    for j in range(self.rows):
                        for i in range(self.cols):
                            timeDelta = self.searchGrid[i][j].visitedTimeStamp - self.search_Run_StartTime
                            self.log_dbug("[" + str(i) + "," + str(j) + \
                                          "] Visit by " + str(self.searchGrid[i][j].visitedBy) + \
                                          " after " + str(timeDelta/1e9) + " secs")

                    self.SEARCH_STATUS = SEARCH_EGRESS

                    # set all ingress/active uav to egress
                    for vehicle in self.searchUAVMap:
                        if self.searchUAVMap[vehicle].status == SEARCH_INGRESS or \
                           self.searchUAVMap[vehicle].status == SEARCH_ACTIVE:
                            self.searchUAVMap[vehicle].status = SEARCH_EGRESS
                            self.searchUAVMap[vehicle].assignedTime = rospy.Time.now()
                            self.wp_msg.lat = self.searchUAVMap[vehicle].originalLLA[0]
                            self.wp_msg.lon = self.searchUAVMap[vehicle].originalLLA[1]
                            self.wp_msg.alt = self.searchUAVMap[vehicle].assignedAltitude
                            if vehicle == self.search_master_searcher_id:
                                self.publishWaypoint(self.wp_msg)

            elif self.SEARCH_STATUS == SEARCH_EGRESS:
                # check if all search slaves has reach originalLLA and set status to search ready
                # and clear search grid
                self._check_timeout()
                egressComplete = True
                for vehicle in self.searchUAVMap:
                    if self.searchUAVMap[vehicle].status == SEARCH_EGRESS:
                        egressComplete = False
                        currentPose = self.searchUAVMap[vehicle].pose
                        tgt_cell = self.searchUAVMap[vehicle].originalLLA

                        if gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1]) < \
                               SEARCH_ARRIVAL_THRESHOLD:
                            self.searchUAVMap[vehicle].status = SEARCH_READY

                if egressComplete == True:
                    self.log_dbug("Swarm Search Master [" + str(self.ownID) + \
                                  "] Master searcher has egressed and all slave searchers deactivated")
                    self.SEARCH_STATUS = SEARCH_READY
                    self.searchGrid = None

                    currentTime = rospy.Time.now()
                    timeDelta = currentTime - self.search_Operation_StartTime
                    self.log_dbug("Swarm Search Master [" + str(self.ownID) + \
                                  "] entire search operation completed" + \
                                  "after " + str(timeDelta/1e9) + " secs")
                    self.SEARCH_STATUS = SEARCH_READY
                    self.searchGrid = None
                    self.ExecuteMasterSearcherBehavior = False
                    self.set_active(False)

            if self.wpmsgQueued == True: #A new network waypoint cmd is triggered this tick
                msg = apmsg.BehaviorParameters()
                msg.id = bytes.SEARCH_WP
                msg.params = self._wps_to_send.pack()
                self._swarmSearchPublisher.publish(msg)
                self._wps_to_send.wp_list = []


    #--------------------------
    # Object-specific functions
    #--------------------------
    def _activate_swarm_search(self, searchAlgoEnum):
        self.log_dbug("Swarm Search Master [" + str(self.ownID) + \
                      "] generate Search Grid:")
        # Create the searchGrid
        self.searchGrid = [[cell() for j in range(self.rows)] for i in range(self.cols)]
        # populate the searchGrid
        for j in range(self.rows):
            for i in range(self.cols):
                self.searchGrid[i][j].grid = [i * SEARCH_CELL_RADIUS, j * SEARCH_CELL_RADIUS]
                self.searchGrid[i][j].LLA = gps_offset(self.bottomLeftLLA[0], \
                                                       self.bottomLeftLLA[1], \
                                                       i * SEARCH_CELL_RADIUS, \
                                                       j * SEARCH_CELL_RADIUS)
                self.log_dbug("[" + str(i) + "," + str(j) + \
                              "] Grid: " + str(self.searchGrid[i][j].grid) + \
                              " LLA: " + str(self.searchGrid[i][j].LLA))
        self.centerofSearchGridLLA = self.searchGrid[self.cols/2][self.rows/2].LLA
        if searchAlgoEnum == 2: # Preplanned swarm search selected
            self.numOfSearcher = 0
            self.rowSearch = False
            for vehicle in self.searchUAVMap:
                self.searchUAVMap[vehicle].preplannedPath = self.numOfSearcher
                self.numOfSearcher += 1
            if self.numOfSearcher < self.rows and self.numOfSearcher < self.cols:
                self.log_dbug("Swarm Search Master ["+str(self.ownID)+"]" + \
                              " Preplanned swarm search algo requires searchers to be at least " + \
                              "the number of rows or columns of the Search Grid:")
                self.log_warn("Swarm Search Master ["+str(self.ownID)+"] " + \
                              "Defaulting to greedy swarm search algo.")
            else:
                self.prePlannedPathPlanned = 0
                self.selectedSearchAlgo = 2
                if self.rows > self.cols:
                    if self.numOfSearcher >= self.rows:
                        self.rowSearch = True
                    else:
                        self.rowSearch = False
                else:
                    if self.numOfSearcher >= self.cols:
                        self.rowSearch = False
                    else:
                        self.rowSearch = True
                if self.rowSearch == True:
                    self.log_dbug("Swarm Search Master ["+str(self.ownID) + \
                                  "] Preplanned swarm search algo activated with " + \
                                  str(self.rows)+" rows search")
                    self.prePlannedPathPlanned = self.rows
                else:
                    self.log_dbug("Swarm Search Master ["+str(self.ownID) + \
                                  "] Preplanned swarm search algo activated with " + \
                                  str(self.cols)+" columns search")
                    self.prePlannedPathPlanned = self.cols


    def _assign_ready_UAVs_to_searchGrid(self):
        # assign all "search_ready" uav to the nearest cell in the search area
        # loop through all ready UAV: from swarm tracker
        # assign search_read uav to center of search grid
        # set each UAV status
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status == SEARCH_READY:
                self.searchUAVMap[vehicle].status = SEARCH_INGRESS
                if self.searchUAVMap[vehicle].originalLLA == None:
                    self.searchUAVMap[vehicle].originalLLA = \
                        self.searchUAVMap[vehicle].pose
                self.searchUAVMap[vehicle].assignedLLA = \
                    self.centerofSearchGridLLA
                self.searchUAVMap[vehicle].assignedCell = \
                   [self.cols/2, self.rows/2]
                self.searchUAVMap[vehicle].assignedTime = rospy.Time.now()
                self.searchUAVMap[vehicle].timeoutdistance = \
                    gps_distance(self.searchUAVMap[vehicle].pose[0], \
                                 self.searchUAVMap[vehicle].pose[1], \
                                 self.centerofSearchGridLLA[0], \
                                 self.centerofSearchGridLLA[1])
                self.searchUAVMap[vehicle].timeoutTime = rospy.Time.now()
                if vehicle == self.search_master_searcher_id:
                    self.wp_msg.lat = self.centerofSearchGridLLA[0]
                    self.wp_msg.lon = self.centerofSearchGridLLA[1]
                    self.wp_msg.alt = self.searchUAVMap[vehicle].assignedAltitude
                    self.publishWaypoint(self.wp_msg)

                else:
                    wp = ( self.centerofSearchGridLLA[0], self.centerofSearchGridLLA[1], \
                               self.searchUAVMap[vehicle].assignedAltitude, vehicle, \
                               self.cols/2, self.rows/2 )
                    self._wps_to_send.wp_list.append(wp)
                    self.wpmsgQueued = True
                self.log_dbug("Swarm Search node: Searcher " + \
                              str(vehicle) + "] ingress to search area")               


    def _assign_ingress_UAVs_to_search(self):

        # loop through all ingress UAV: from swarm tracker
        # assign search_read uav to the cell
            #(Set altitude to SEARCH_ALTITUDE
            # set each UAV status and assignedTime "ross::Time::now()"
        reached = False
        currentTime = rospy.Time.now()
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status == SEARCH_INGRESS:
                currentPose = self.searchUAVMap[vehicle].pose
                for j in range(self.rows):
                    for i in range(self.cols):
                        if reached == False:
                            tgt_cell = self.searchGrid[i][j].LLA
                            if gps_distance(currentPose[0], currentPose[1], \
                                            tgt_cell[0], tgt_cell[1]) < SEARCH_ARRIVAL_THRESHOLD:
                                reached = True
                                self.searchUAVMap[vehicle].status = SEARCH_ACTIVE
                                if self.selectedSearchAlgo == 2:
                                    if self.searchUAVMap[vehicle].preplannedPath < self.prePlannedPathPlanned:
                                        if self.rowSearch == True:
                                            tgt_cell = self.searchGrid[0][self.searchUAVMap[vehicle].preplannedPath].LLA
                                            tgt_cell2 = self.searchGrid[self.cols-1][self.searchUAVMap[vehicle].preplannedPath].LLA
                                            if gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1]) \
                                               < gps_distance(currentPose[0], currentPose[1], tgt_cell2[0], tgt_cell2[1]):
                                                self.searchUAVMap[vehicle].assignedCell = [0,self.searchUAVMap[vehicle].preplannedPath]
                                                self.log_dbug("Swarm Search node: Searcher ["+ \
                                                              str(vehicle)+"] assigned entry via: ["+str(0) + "," + \
                                                              str(self.searchUAVMap[vehicle].preplannedPath) + "]")
                                            else:
                                                tgt_cell = self.searchGrid[self.cols-1][self.searchUAVMap[vehicle].preplannedPath].LLA
                                                self.searchUAVMap[vehicle].assignedCell = [self.cols-1, self.searchUAVMap[vehicle].preplannedPath]
                                                self.log_dbug("Swarm Search node: Searcher [" + str(vehicle) + \
                                                              "] assigned entry via: [" + str(self.cols-1) + \
                                                              "," + str(self.searchUAVMap[vehicle].preplannedPath) + "]")
                                        else:
                                            tgt_cell = self.searchGrid[self.searchUAVMap[vehicle].preplannedPath][0].LLA
                                            tgt_cell2 = self.searchGrid[self.searchUAVMap[vehicle].preplannedPath][self.rows-1].LLA
                                            if gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1]) \
                                               < gps_distance(currentPose[0], currentPose[1], tgt_cell2[0], tgt_cell2[1]):
                                                self.searchUAVMap[vehicle].assignedCell = [self.searchUAVMap[vehicle].preplannedPath,0]
                                                self.log_dbug("Swarm Search node: Searcher [" + \
                                                              str(vehicle) + "] assigned entry via: ["+ \
                                                              str(self.searchUAVMap[vehicle].preplannedPath) + "," + str(0) + "]")
                                            else:
                                                tgt_cell = self.searchGrid[self.searchUAVMap[vehicle].preplannedPath][self.rows-1].LLA
                                                self.searchUAVMap[vehicle].assignedCell = [self.searchUAVMap[vehicle].preplannedPath, self.rows-1]
                                                self.log_dbug("Swarm Search node: Searcher [" + str(vehicle) + \
                                                              "] assigned entry via: [" + \
                                                              str(self.searchUAVMap[vehicle].preplannedPath) + "," + str(self.rows-1) + "]")
                                    else: #terminate extra searchers
                                        self.searchUAVMap[vehicle].assignedCell = [254,254]
                                        tgt_cell = self.searchUAVMap[vehicle].originalLLA
                                        self.searchUAVMap[vehicle].status = SEARCH_EGRESS
                                        self.log_dbug("Swarm Search node: Extra searcher ["+str(vehicle) + \
                                                      "] removed from search")
                                else:
                                    self.searchUAVMap[vehicle].assignedCell = [i,j]
                                    self.log_dbug("Swarm Search node: Searcher [" + str(vehicle) + \
                                                  "] assigned entry via: [" + str(i) + "," + str(j) + "]")
                                self.searchUAVMap[vehicle].assignedTime = currentTime
                                self.searchUAVMap[vehicle].assignedLLA = tgt_cell
                                self.searchUAVMap[vehicle].timeoutdistance = \
                                    gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1])
                                self.searchUAVMap[vehicle].timeoutTime = currentTime
                                #assign UAV to tgt_cell
                                if vehicle == self.search_master_searcher_id:
                                    self.wp_msg.lat = tgt_cell[0]
                                    self.wp_msg.lon = tgt_cell[1]
                                    self.wp_msg.alt = self.searchUAVMap[vehicle].assignedAltitude
                                    self.publishWaypoint(self.wp_msg)

                                else:
                                    rec = self.searchUAVMap[vehicle]
                                    wp = ( tgt_cell[0], tgt_cell[1], \
                                           rec.assignedAltitude, vehicle, \
                                           rec.assignedCell[0], rec.assignedCell[1] )
                                    self._wps_to_send.wp_list.append(wp)
                                    self.wpmsgQueued = True
        return reached


    def _check_search_operation(self):
        runOutofCells=False
        currentTime = rospy.Time.now()

        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status == SEARCH_ACTIVE:
                currentPose = self.searchUAVMap[vehicle].pose
                tgt_cell = \
                    self.searchGrid[self.searchUAVMap[vehicle].\
                         assignedCell[0]][self.searchUAVMap[vehicle].\
                         assignedCell[1]].LLA

                if gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1]) < SEARCH_CELL_THRESHOLD:
                    if self.search_Run_firstSearch == False:
                        self.search_Run_firstSearch = True
                        timeDelta = currentTime - self.search_Run_StartTime
                        self.search_Run_StartTime = currentTime
                        self.log_dbug("Swarm Search Master ["+str(self.ownID)+" actual search started after "+ \
                                     str(timeDelta/1e9) + " secs")
                    if self.searchGrid[self.searchUAVMap[vehicle].\
                            assignedCell[0]][self.searchUAVMap[vehicle].\
                            assignedCell[1]].visited == False:
                        self.log_dbug("Swarm Search node: Searcher [" + \
                                      str(vehicle) + "] searched cell " + \
                                      str(self.searchUAVMap[vehicle].assignedCell))
                        self.searchGrid[self.searchUAVMap[vehicle].\
                             assignedCell[0]][self.searchUAVMap[vehicle].\
                             assignedCell[1]].visited = True
                        self.searchGrid[self.searchUAVMap[vehicle].\
                             assignedCell[0]][self.searchUAVMap[vehicle].\
                             assignedCell[1]].visitedBy = vehicle
                        self.searchGrid[self.searchUAVMap[vehicle].\
                             assignedCell[0]][self.searchUAVMap[vehicle].\
                             assignedCell[1]].visitedTimeStamp = rospy.Time.now()

                    else:
                        self.log_dbug("Swarm Search node: Searcher [" + \
                                      str(vehicle) + "] visited cell " + \
                                      str(self.searchUAVMap[vehicle].assignedCell))
                    result = self._assign_search_UAVs_to_nextCell(vehicle)

                    if result[0] == SEARCH_CELL_STILL_AVAILABLE: #assign UAV to tgt_cell
                        self.searchUAVMap[vehicle].assignedCell = [result[1],result[2]]
                        self.searchUAVMap[vehicle].assignedTime = currentTime
                        self.searchGrid[result[1]][result[2]].assigned = True
                        self.searchGrid[result[1]][result[2]].assignedTo = vehicle
                        self.searchUAVMap[vehicle].assignedLLA = self.searchGrid[result[1]][result[2]].LLA
                        self.searchUAVMap[vehicle].timeoutdistance = \
                            gps_distance(currentPose[0], currentPose[1], \
                                         self.searchGrid[result[1]][result[2]].LLA[0], \
                                         self.searchGrid[result[1]][result[2]].LLA[1])
                        self.searchUAVMap[vehicle].timeoutTime = currentTime
                        self.log_dbug("Swarm Search node: Searcher [" + \
                                      str(vehicle) + "] assigned cell " + \
                                      str(self.searchUAVMap[vehicle].assignedCell))

                        if vehicle == self.search_master_searcher_id:
                            self.wp_msg.lat = self.searchGrid[result[1]][result[2]].LLA[0]
                            self.wp_msg.lon = self.searchGrid[result[1]][result[2]].LLA[1]
                            self.wp_msg.alt = self.searchUAVMap[vehicle].assignedAltitude
                            self.publishWaypoint(self.wp_msg)

                        else:
                            grid = self.searchGrid[result[1]][result[2]]
                            wp = ( grid.LLA[0], grid.LLA[1], \
                                   self.searchUAVMap[vehicle].assignedAltitude, \
                                   vehicle, result[1], result[2] )
                            self._wps_to_send.wp_list.append(wp)
                            self.wpmsgQueued = True

                    else:
                        if result[0] == SEARCH_CELL_FULLY_VISITED:
                            runOutofCells = True
                        self.searchUAVMap[vehicle].status = SEARCH_EGRESS
                        self.searchUAVMap[vehicle].assignedTime = currentTime
                        self.searchUAVMap[vehicle].assignedLLA = self.searchUAVMap[vehicle].originalLLA
                        self.searchUAVMap[vehicle].timeoutdistance = \
                            gps_distance(currentPose[0], currentPose[1], \
                                         self.searchUAVMap[vehicle].originalLLA[0], \
                                         self.searchUAVMap[vehicle].originalLLA[1])
                        self.searchUAVMap[vehicle].timeoutTime = currentTime
                        self.log_dbug("Swarm Search node: Searcher [" + \
                                      str(vehicle) + "] returning home")

                        if vehicle == self.search_master_searcher_id:
                            self.wp_msg.lat = self.searchUAVMap[vehicle].originalLLA[0]
                            self.wp_msg.lon = self.searchUAVMap[vehicle].originalLLA[1]
                            self.wp_msg.alt = self.searchUAVMap[vehicle].assignedAltitude
                            self.publishWaypoint(self.wp_msg)

                        else:
                            rec = self.searchUAVMap[vehicle]
                            wp = ( rec.originalLLA[0], rec.originalLLA[1], \
                                   rec.assignedAltitude, vehicle, 254, 254 )
                            self._wps_to_send.wp_list.append(wp)
                            self.wpmsgQueued = True
        return runOutofCells


    def _assign_search_UAVs_to_nextCell(self, vehicleID):
        # assign new unvisited cell if possible
        cellResult = SEARCH_CELL_FULLY_VISITED
        currentCol = self.searchUAVMap[vehicleID].assignedCell[0]
        currentRow = self.searchUAVMap[vehicleID].assignedCell[1]
        tgt_cell = [0,0]
        distance = SEARCH_MAX_THRESHOLD
        currentPose = self.searchUAVMap[vehicleID].pose
        if self.selectedSearchAlgo == 2:
            if self.rowSearch == True: # find next closest cell in the same row
                for i in range(self.cols):
                    if self.searchGrid[i][currentRow].visited == False:
                        if cellResult == SEARCH_CELL_FULLY_VISITED:
                            cellResult = SEARCH_CELL_FULLY_ASSIGNED
                        if self.searchGrid[i][currentRow].assigned == False:
                            cellResult = SEARCH_CELL_STILL_AVAILABLE
                            cell = self.searchGrid[i][currentRow].LLA
                            tempDist=gps_distance(currentPose[0], currentPose[1], cell[0], cell[1])
                            if tempDist < distance:
                                distance = tempDist
                                tgt_cell = [i,currentRow]
            else:
                for j in range(self.rows): # find next closest cell in the same col
                    if self.searchGrid[currentCol][j].visited == False:
                        if cellResult == SEARCH_CELL_FULLY_VISITED:
                            cellResult = SEARCH_CELL_FULLY_ASSIGNED
                        if self.searchGrid[currentCol][j].assigned == False:
                            cellResult = SEARCH_CELL_STILL_AVAILABLE
                            cell = self.searchGrid[currentCol][j].LLA
                            tempDist=gps_distance(currentPose[0], currentPose[1], cell[0], cell[1])
                            if tempDist < distance:
                                distance = tempDist
                                tgt_cell = [currentCol,j]
            if cellResult == SEARCH_CELL_FULLY_VISITED: # To check if the whole search grid is fully visited
                for j in range(self.rows):
                    for i in range(self.cols):
                        if self.searchGrid[i][j].visited == False:
                            cellResult = SEARCH_CELL_FULLY_ASSIGNED
        else:
            for j in range(self.rows):
                for i in range(self.cols):
                    if self.searchGrid[i][j].visited == False:
                        if cellResult == SEARCH_CELL_FULLY_VISITED:
                            cellResult = SEARCH_CELL_FULLY_ASSIGNED
                        if self.searchGrid[i][j].assigned == False:
                            cellResult = SEARCH_CELL_STILL_AVAILABLE
                            cell = self.searchGrid[i][j].LLA
                            tempDist=gps_distance(currentPose[0], currentPose[1], cell[0], cell[1])
                            if tempDist < distance:
                                distance = tempDist
                                tgt_cell = [i,j]
        if cellResult == SEARCH_CELL_FULLY_VISITED:
            return [SEARCH_CELL_FULLY_VISITED,0,0]

        else:
            return [cellResult, tgt_cell[0], tgt_cell[1]]


    def _check_timeout(self):
        # scan through all UAVs, if any did not get closer in the last interval, resend LLA
        currentTime = rospy.Time.now()
        for vehicle in self.searchUAVMap:
            #any states that has assigned waypoints
            if self.searchUAVMap[vehicle].status in (SEARCH_INGRESS, SEARCH_ACTIVE, SEARCH_EGRESS):
                # timeout interval up
                if self.searchUAVMap[vehicle].timeoutTime + SEARCH_TIMEOUT_INTERVAL < currentTime:
                    distFromTgt = gps_distance(self.searchUAVMap[vehicle].pose[0], \
                                               self.searchUAVMap[vehicle].pose[1], \
                                               self.searchUAVMap[vehicle].assignedLLA[0], \
                                               self.searchUAVMap[vehicle].assignedLLA[1])

                    #uav is closer since last check. Update latest distance and time
                    if distFromTgt < self.searchUAVMap[vehicle].timeoutdistance:
                        if self.searchUAVMap[vehicle].status == SEARCH_EGRESS:
                            self.searchUAVMap[vehicle].timeoutdistance = \
                                distFromTgt + SEARCH_TIMEOUT_LOITERDISTANCEBUFFER

                        else:
                            self.searchUAVMap[vehicle].timeoutdistance = distFromTgt
                        self.searchUAVMap[vehicle].timeoutTime = currentTime
                        self.searchUAVMap[vehicle].timeoutCounter = 0

                    else:
                        # resend assigned lla
                        if self.searchUAVMap[vehicle].timeoutCounter < SEARCH_TIMEOUT_STRIKES:
                            self.searchUAVMap[vehicle].timeoutCounter += 1
                            self.log_dbug("Swarm Search node: TIMEOUT " + \
                                          str(self.searchUAVMap[vehicle].timeoutCounter) + \
                                          " for UAV [" + str(vehicle) + "] Resend LLA...")
                            self.searchUAVMap[vehicle].timeoutTime = currentTime

                            if vehicle == self.search_master_searcher_id:
                                self.wp_msg.lat = self.searchUAVMap[vehicle].assignedLLA[0]
                                self.wp_msg.lon = self.searchUAVMap[vehicle].assignedLLA[1]
                                self.wp_msg.alt = self.searchUAVMap[vehicle].assignedAltitude
                                self.publishWaypoint(self.wp_msg)

                            else:
                                rec = self.searchUAVMap[vehicle]
                                wp = ( rec.assignedLLA[0], rec.assignedLLA[1], \
                                       rec.assignedAltitude, vehicle, \
                                       rec.assignedCell[0], rec.assignedCell[1] )
                                self._wps_to_send.wp_list.append(wp)
                                self.wpmsgQueued = True

                        else: # UAV not getting closer after timeout strikes, free assigned cell and place uav out of search operation
                            self.log_dbug("Swarm Search node: UAV [" + str(vehicle) + \
                                          "] STRIKEOUT!")
                            if self.searchUAVMap[vehicle].status == SEARCH_ACTIVE:
                                # see if another UAV active. if so, they can take over the released cell
                                if self._available_activeUAVHandover(vehicle) == True:
                                    self.searchGrid[self.searchUAVMap[vehicle].\
                                         assignedCell[0]][self.searchUAVMap[vehicle].\
                                         assignedCell[1]].assigned = False
                                    self.searchGrid[self.searchUAVMap[vehicle].\
                                         assignedCell[0]][self.searchUAVMap[vehicle].\
                                         assignedCell[1]].assignedTo = None
                                    self.log_dbug("Search cell " + \
                                                  str(self.searchUAVMap[vehicle].assignedCell) + \
                                                  "  released for reassignment")
                                else:
                                    egressUAVID = \
                                        self._closest_egressUAVHandover(self.searchUAVMap[vehicle].assignedCell[0], \
                                                                        self.searchUAVMap[vehicle].assignedCell[1])

                                    if egressUAVID != 0: #Found egress UAV to take over
                                        self.searchUAVMap[egressUAVID].status = SEARCH_ACTIVE
                                        self.searchUAVMap[egressUAVID].assignedCell = \
                                            self.searchUAVMap[vehicle].assignedCell
                                        self.searchUAVMap[egressUAVID].assignedTime = currentTime
                                        self.searchGrid[self.searchUAVMap[vehicle].\
                                             assignedCell[0]][self.searchUAVMap[vehicle].\
                                             assignedCell[1]].assigned = True
                                        self.searchGrid[self.searchUAVMap[vehicle].\
                                             assignedCell[0]][self.searchUAVMap[vehicle].\
                                             assignedCell[1]].assignedTo = egressUAVID
                                        self.searchUAVMap[egressUAVID].assignedLLA = \
                                            self.searchGrid[self.searchUAVMap[vehicle].\
                                            assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].LLA
                                        self.searchUAVMap[egressUAVID].timeoutdistance = \
                                            gps_distance(self.searchUAVMap[egressUAVID].pose[0], \
                                                         self.searchUAVMap[egressUAVID].pose[1], \
                                                         self.searchUAVMap[egressUAVID].assignedLLA[0], \
                                                         self.searchUAVMap[egressUAVID].assignedLLA[1])
                                        self.searchUAVMap[egressUAVID].timeoutTime = currentTime
                                        self.log_dbug("Swarm Search node: Egressed Searcher [" + \
                                                      str(egressUAVID) + "] assigned to take over strikeout [" + \
                                                      str(vehicle) + "] assigned cell " + \
                                                      str(self.searchUAVMap[vehicle].assignedCell))

                                        if egressUAVID == self.search_master_searcher_id:
                                            self.wp_msg.lat = self.searchUAVMap[egressUAVID].assignedLLA[0]
                                            self.wp_msg.lon = self.searchUAVMap[egressUAVID].assignedLLA[1]
                                            self.wp_msg.alt = self.searchUAVMap[egressUAVID].assignedAltitude
                                            self.publishWaypoint(self.wp_msg)

                                        else:
                                            rec = self.searchUAVMap[egressUAVID]
                                            wp = ( rec.assignedLLA[0], rec.assignedLLA[1], \
                                                   rec.assignedAltitude, egressUAVID, \
                                                   rec.assignedCell[0], rec.assignedCell[1] )
                                            self._wps_to_send.wp_list.append(wp)
                                            self.wpmsgQueued = True

                                    else:
                                        self.log_dbug("Search FAILED!  cell " + \
                                                      str(self.searchUAVMap[vehicle].assignedCell) + \
                                                      " released for assignment with no UAV to take over")
                            self.searchUAVMap[vehicle].status = SEARCH_FAULT


    def _available_activeUAVHandover(self, faultyUAVID):
        available = False
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status in (SEARCH_INGRESS, SEARCH_ACTIVE):
                if vehicle != faultyUAVID:
                    available = True
                    break
        return available


    def _closest_egressUAVHandover(self, cell_x, cell_y):
        closestUAV = 0
        closestDistance = SEARCH_MAX_THRESHOLD
        tgtLLA = self.searchGrid[cell_x][cell_y].LLA
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status == SEARCH_EGRESS:
                distFromTgt = gps_distance(self.searchUAVMap[vehicle].pose[0], \
                                           self.searchUAVMap[vehicle].pose[1], \
                                           tgtLLA[0], tgtLLA[0])
                if distFromTgt < closestDistance:
                    closestDistance = distFromTgt
                    closestUAV = vehicle
        return closestUAV


    #------------------------------------------
    # ROS Subscriber callbacks -for this object
    #------------------------------------------
    # Handle incoming swarm_uav_states messages
    # @param swarmMsg: message containing swarm data (SwarmStateStamped)
    def _process_swarm_uav_states(self, swarmMsg):


        # DTD NOTE:  This method overrides the parent class method, which 
        # probably isn't a good idea, but it would require a significant class
        # rewrite to fix it.  For now, the parent class functionality is called
        # from here so that both the parent and child class functions run.
        super(SwarmSearcher, self)._process_swarm_uav_states(swarmMsg)


        for vehicle in swarmMsg.swarm:
            if vehicle.vehicle_id in self.searchUAVMap:
                self.searchUAVMap[vehicle.vehicle_id].pose = \
                    [vehicle.state.pose.pose.position.lat, vehicle.state.pose.pose.position.lon]
                self.searchUAVMap[vehicle.vehicle_id].subSwarmID = vehicle.subswarm_id

                if vehicle.swarm_state != self.searchUAVMap[vehicle.vehicle_id].receivedState:
                    if self.ExecuteMasterSearcherBehavior == True:
                        if self.searchUAVMap[vehicle.vehicle_id].receivedState == \
                               SEARCH_SWARM_SEARCH_ACTIVE_STATE and \
                           vehicle.swarm_state == SEARCH_SWARM_SEARCH_READY_STATE:
                            self.log_dbug("UAV [" + str(vehicle.vehicle_id) + \
                                          "] terminated swarm behaviour.")
                            self.searchUAVMap[vehicle.vehicle_id].status = SEARCH_FAULT

                            if self.ownID == vehicle.vehicle_id:
                                currentTime = rospy.Time.now()
                                timeDelta = currentTime - self.search_Operation_StartTime
                                self.log_dbug("Search Operation SUSPENDED after " + \
                                              str(timeDelta/1e9) + " secs")

                                for j in range(self.rows):
                                    for i in range(self.cols):
                                        if self.searchGrid[i][j].visited == False:
                                            self.log_dbug("[" + str(i) + "," + str(j) + \
                                                          "] was not visited.")

                                        else:
                                            timeDelta = self.searchGrid[i][j].visitedTimeStamp - \
                                                        self.search_Run_StartTime
                                            self.log_dbug("[" + str(i) + "," + str(j) + \
                                                          "] Visit by " + \
                                                          str(self.searchGrid[i][j].visitedBy) + \
                                                          " after " + str(timeDelta/1e9) + " secs")

                                self.SEARCH_STATUS = SEARCH_READY
                                self.searchGrid = None
                                self.ExecuteMasterSearcherBehavior = False
                                self.set_active(False)

                    self.searchUAVMap[vehicle.vehicle_id].receivedState = vehicle.swarm_state
                    if self.searchUAVMap[vehicle.vehicle_id].receivedState == SEARCH_SWARM_SEARCH_ACTIVE_STATE:
                        # When swarm behaviour just turn active, the first local state is search ready
                        self.searchUAVMap[vehicle.vehicle_id].status = SEARCH_READY

            else:
                self.searchUAVMap[vehicle.vehicle_id] = searchUAVdata()
                self.searchUAVMap[vehicle.vehicle_id].pose = \
                    [vehicle.state.pose.pose.position.lat, vehicle.state.pose.pose.position.lon]
                self.searchUAVMap[vehicle.vehicle_id].subSwarmID = vehicle.subswarm_id
                self.searchUAVMap[vehicle.vehicle_id].assignedAltitude = \
                    SEARCH_ALTITUDE + (self.uavAltitudeCounter * SEARCH_SAFETY_ALTITUDE_INTERVAL)
                self.uavAltitudeCounter += 1


    def _process_swarm_data_msg(self, swarmData):
        # Only processes waypoint messages and only if active
        if not self.is_active or \
               swarmData.id != bytes.SEARCH_WP:
            return

        parser = bytes.SearchWaypointParser()
        parser.unpack(swarmData.params)

        # Tuples of the form: (lat, lon, rel_alt, vid, cell_x, cell_y)
        for wp in parser.wp_list:
            if self.ownID == wp[3]:
                if wp[4] >= 254 and wp[5] >= 254:
                    self.set_active(False)
                    self.log_info("Swarm Searcher [" + str(self.ownID)+ \
                                  "] proceeding to deactivate")

                else:
                    self.wp_msg.lat = wp[0]
                    self.wp_msg.lon = wp[1]
                    self.wp_msg.alt = self.searchUAVMap[self.ownID].assignedAltitude
                    self.publishWaypoint(self.wp_msg)
                    self.log_dbug("Swarm Searcher " + str(self.ownID) + \
                                  "] proceeding to assigned cell [" + \
                                  str(wp[4]) + ", " + str(wp[5]) + "]")

            else:
                if wp[4] >= 254 and wp[5] >= 254:
                    self.log_dbug("Swarm Search " + str(self.ownID) + \
                                  " node: Received network wp cmd for Slave Searcher" + \
                                  str(wp[3]) + "] to deactivate")


#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    searcher = SwarmSearcher("swarm_searcher",rospy.get_param("aircraft_id"),args)
    searcher.runAsNode(10)
