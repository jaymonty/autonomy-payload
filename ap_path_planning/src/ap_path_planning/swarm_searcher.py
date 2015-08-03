#!/usr/bin/env python

# Swarm_searcher code implemented as a node.
# Improved version written by Stefan Jorgensen, August 2014
#
# Modified by Duane Davis (object implementation)
# Modified by Dylan Lau (Swarm_searcher impementation as object)


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
import ap_lib.nodeable as nodeable
import ap_lib.waypoint_controller as wp_controller
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
SEARCH_CELL_THRESHOLD = 75 # Distance to determine that cell is searched (must be greater than infinite loiter radius
SEARCH_ARRIVAL_THRESHOLD = 150 # Distance to start assigning cells in search grid to UAV
SEARCH_MAX_THRESHOLD = 100000000 # Max number for comparsion
SEARCH_TIMEOUT_INTERVAL = 15 # 15 secs interval to check if searcher get closer to assigned waypoint
SEARCH_TIMEOUT_STRIKES = 3 # Number of strikes a UAV can get from timeout before being thrown out of the search operation
SEARCH_TIMEOUT_LOITERDISTANCEBUFFER = 10 # Buffer for UAV doing loiter such as Egressing after reaching the swarming waypoint

SEARCH_SWARM_SEARCH_READY_STATE = enums.SWARM_READY # enum state when UAV is ready for swarm behavior
SEARCH_SWARM_SEARCH_ACTIVE_STATE = enums.SWARM_ACTIVE # enum state when UAV is executing an active swarm behavior

# "struct" to store data for search search cell
class cell():
    LLA=None
    grid=None
    assigned=False
    assignedTo=None
    visited=False
    visitedBy=None
    visitedTimeStamp=None

# "struct" for Waypoint Msg
class wpMsg():
    recipientvehicle_id = None
    waypoint = apbrgmsg.LLA()
    searchCell_x = None
    searchCell_y = None

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
class SwarmSearcher(wp_controller.WaypointController):


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

    # SearchSubSwarmID
    searchSubSwarmID = 0

    numOfRunsToDo = 0 # 0 means unlimited
    numOfRunsDone = 0 #

    rel_Altitude = None # Global storage for Relative Altitude of the UAV

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
        wp_controller.WaypointController.__init__(self, nodename, \
            enums.SWARM_SEARCH_CTRLR)
        self.DBUG_PRINT = False
        self.WARN_PRINT = False
        # Boolean flag to determine Centralised search node
        self.SEARCH_MASTER = False
        self.SEARCH_STATUS = SEARCH_READY
        self.ownID = ownAC
        self.ExecuteMasterSearcherBehavior = False

        # Counter used to assign safety altitude to each UAV
        self.uavAltitudeCounter = 0

        # Time when search operation begins
        self.search_Operation_StartTime = None
        # Time when current search runs begins
        self.search_Run_StartTime = None

        # Boolean flag to determine if any waypoint message to send
        self.wpmsgQueued = False

        self._swarmSearchPublisher = None
        self._assignedSearchMessage = apmsg.SwarmSearchWaypointList()


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Establishes the callbacks for the FollowController object.  The object
    # subscribes to the swarm_tracker/swarm_state topic for own-aircraft
    # and follow aircraft state updates
    # @param params: no additional parameters required by this method
    def callbackSetup(self, params=[]):
        self.createSubscriber("swarm_uav_states", apmsg.SwarmStateStamped, \
                              self._process_swarm_uav_states)
        self.createSubscriber("recv_swarm_search_waypoint", apmsg.SwarmSearchWaypointList, \
                                self._process_swarm_search_waypoint)
        self.createSubscriber("swarmSearch_setup", apmsg.SwarmSearchOrderStamped, \
                                self._process_swarmSearch_setup)
        self.createSubscriber("acs_pose", apbrgmsg.Geodometry, \
                              self.sub_swarm_relAlt_update)
    def publisherSetup(self, params=[]):
        self._swarmSearchPublisher = \
            self.createPublisher("send_swarm_search_waypoint", apmsg.SwarmSearchWaypointList, 1)
        self._searchDeactivatePublisher = \
            self.createPublisher("deactivate_swarm_search", stdmsg.UInt8, 1, True)

    def serviceSetup(self, params=[]):
        pass

    def runController(self):
        if self.ExecuteMasterSearcherBehavior == True:
            self.wpmsgQueued = False #flag to determine if any waypoint message to send
            del self._assignedSearchMessage.waypoints[:]    # Clear current message contents

            #for vehicle in self.searchUAVMap:
            #    print "\033[94m TimeLoop: UAV \033[96m[" + str(vehicle) + "] \033[94m Received State ["+str(self.searchUAVMap[vehicle].receivedState)+"] pose: \033[0m" + str(self.searchUAVMap[vehicle].pose)
                
            if self.SEARCH_STATUS == SEARCH_READY:
                if self.numOfRunsToDo == 0 or self.numOfRunsDone < self.numOfRunsToDo:
                    self.SEARCH_STATUS = SEARCH_INGRESS
                    self.search_Run_StartTime = time.time()
                    print "\033[94mSwarm Search Master \033[96m["+str(self.ownID)+"]\033[94m current search run \033[96m"+str(self.numOfRunsDone+1)+"\033[94m started on \033[96m"+str(time.asctime(time.localtime(time.time())))+"\033[0m"
            elif self.SEARCH_STATUS == SEARCH_INGRESS:
                self._assign_ready_UAVs_to_searchGrid()
                self._check_timeout()
                # Check if any of the search uav has reached the search area. set search status to active if true
                reached = self._assign_ingress_UAVs_to_search()

                if reached == True:
                    currentTime = time.time()
                    timeDelta = currentTime - self.search_Run_StartTime
                    print "\033[94mSwarm Search Master \033[96m["+str(self.ownID)+"]\033[94m actual search started after \033[96m"+"{0:.1f}".format(timeDelta)+"\033[94m secs\033[0m"
                    self.SEARCH_STATUS = SEARCH_ACTIVE
            elif self.SEARCH_STATUS == SEARCH_ACTIVE:
                self._assign_ready_UAVs_to_searchGrid()
                self._assign_ingress_UAVs_to_search()
                self._check_timeout()

                # Check if search is completed (all cells visited) set status to SEARCH_EGRESS
                searchComplete = self._check_search_operation()

                if searchComplete == True:
                    self.numOfRunsDone += 1
                    currentTime = time.time()
                    timeDelta = currentTime - self.search_Run_StartTime
                    print "\033[94mSwarm Search Master \033[96m["+str(self.ownID)+"]\033[94m current search run \033[96m"+str(self.numOfRunsDone)+" \033[94mcompleted after \033[96m"+"{0:.1f}".format(timeDelta)+"\033[94m secs\033[0m"
                    for j in range(self.rows):
                        for i in range(self.cols):
                            timeDelta = self.searchGrid[i][j].visitedTimeStamp - self.search_Run_StartTime
                            print "\033[93m [" + str(i) + "," + str(j) + "] \033[94mVisit by \033[96m"+ str(self.searchGrid[i][j].visitedBy) + "\033[94m after \033[96m"+"{0:.1f}".format(timeDelta)+"\033[94m secs\033[0m"
                    self.SEARCH_STATUS = SEARCH_EGRESS
                    # set all ingress/active uav to egress
                    for vehicle in self.searchUAVMap:
                        if self.searchUAVMap[vehicle].status == SEARCH_INGRESS or self.searchUAVMap[vehicle].status == SEARCH_ACTIVE:
                            self.searchUAVMap[vehicle].status = SEARCH_EGRESS
                            self.searchUAVMap[vehicle].assignedTime = time.time()
                            lla = apbrgmsg.LLA()
                            lla.lat = self.searchUAVMap[vehicle].originalLLA[0]
                            lla.lon = self.searchUAVMap[vehicle].originalLLA[1]
                            lla.alt = self.searchUAVMap[vehicle].assignedAltitude
                            if vehicle == self.search_master_searcher_id:
                                self.publishWaypoint(lla)

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
                        if gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1]) < SEARCH_ARRIVAL_THRESHOLD:
                            if vehicle != self.search_master_searcher_id: 
                                _wpMsg = wpMsg()
                                _wpMsg.recipientvehicle_id = vehicle
                                _wpMsg.waypoint.lat = self.searchUAVMap[vehicle].originalLLA[0]
                                _wpMsg.waypoint.lon = self.searchUAVMap[vehicle].originalLLA[1]
                                _wpMsg.waypoint.alt = self.searchUAVMap[vehicle].assignedAltitude
                                _wpMsg.searchCell_x = 255
                                _wpMsg.searchCell_y = 255
                                self._assignedSearchMessage.waypoints.append(_wpMsg)
                                self.wpmsgQueued = True
                            else:
                                self.searchUAVMap[vehicle].status = SEARCH_READY

                if egressComplete == True:
                    print "\033[94mSwarm Search Master \033[96m["+str(self.ownID)+"]\033[94m Master searcher has egressed and all slave searchers deactivated\033[0m"
                    self.SEARCH_STATUS = SEARCH_READY
                    self.searchGrid = None
                    if self.numOfRunsDone == self.numOfRunsToDo:
                        currentTime = time.time()
                        timeDelta = currentTime - self.search_Operation_StartTime
                        print "\033[94mSwarm Search Master \033[96m["+str(self.ownID)+"]\033[94m entire search operation completed with \033[96m" + str(self.numOfRunsToDo)+ " \033[94mruns \033[94mafter \033[96m"+"{0:.3f}".format(timeDelta)+"\033[94m secs\033[0m"
                        self.SEARCH_STATUS = SEARCH_READY
                        self.searchGrid = None
                        self.ExecuteMasterSearcherBehavior = False
                        self._searchDeactivatePublisher.publish(1)
                    else:
                        self._activate_swarm_search()

            if self.wpmsgQueued == True: #A new network waypoint cmd is triggered this tick
                self._swarmSearchPublisher.publish(self._assignedSearchMessage)

    #--------------------------
    # Object-specific functions
    #--------------------------
    def _activate_swarm_search(self):
        print "\033[94mSwarm Search Master \033[96m["+str(self.ownID)+"]\033[94m generate Search Grid:\033[0m"

        # Create the searchGrid
        self.searchGrid = [[cell() for j in range(self.rows)] for i in range(self.cols)]

        # populate the searchGrid
        for j in range(self.rows):
            for i in range(self.cols):
                self.searchGrid[i][j].grid = [i * SEARCH_CELL_RADIUS, j * SEARCH_CELL_RADIUS]
                self.searchGrid[i][j].LLA = gps_offset(self.bottomLeftLLA[0], self.bottomLeftLLA[1], i * SEARCH_CELL_RADIUS, j * SEARCH_CELL_RADIUS)
                print "\033[93m [" + str(i) + "," + str(j) + "] \033[36m Grid: "+ str(self.searchGrid[i][j].grid) + " \033[0mLLA: " + str(self.searchGrid[i][j].LLA)
        self.centerofSearchGridLLA = self.searchGrid[self.cols/2][self.rows/2].LLA

        # save Time search start

    def _assign_ready_UAVs_to_searchGrid(self):
        #print "\033[94m" + "Dylan test: SwarmSearcher _Assign any ready UAV to search grid" + "\033[0m"

        # assign all "search_ready" uav to the nearest cell in the search area
        # loop through all ready UAV: from swarm tracker
        # assign search_read uav to center of search grid
        # set each UAV status
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status == SEARCH_READY and self.searchUAVMap[vehicle].subSwarmID == self.searchSubSwarmID:
                self.searchUAVMap[vehicle].status = SEARCH_INGRESS
                if self.searchUAVMap[vehicle].originalLLA == None:
                    self.searchUAVMap[vehicle].originalLLA = self.searchUAVMap[vehicle].pose
                self.searchUAVMap[vehicle].assignedLLA = self.centerofSearchGridLLA
                self.searchUAVMap[vehicle].assignedCell = [self.cols/2, self.rows/2]
                self.searchUAVMap[vehicle].assignedTime = time.time()
                self.searchUAVMap[vehicle].timeoutdistance = gps_distance(self.searchUAVMap[vehicle].pose[0], self.searchUAVMap[vehicle].pose[1], self.centerofSearchGridLLA[0], self.centerofSearchGridLLA[1])
                self.searchUAVMap[vehicle].timeoutTime = time.time()
                if vehicle == self.search_master_searcher_id:
                    lla = apbrgmsg.LLA()
                    lla.lat = self.centerofSearchGridLLA[0]
                    lla.lon = self.centerofSearchGridLLA[1]
                    lla.alt = self.searchUAVMap[vehicle].assignedAltitude
                    self.publishWaypoint(lla)
                else:
                    _wpMsg = wpMsg()
                    _wpMsg.recipientvehicle_id = vehicle
                    _wpMsg.waypoint.lat = self.centerofSearchGridLLA[0]
                    _wpMsg.waypoint.lon = self.centerofSearchGridLLA[1]
                    _wpMsg.waypoint.alt = self.searchUAVMap[vehicle].assignedAltitude
                    _wpMsg.searchCell_x = self.cols/2
                    _wpMsg.searchCell_y = self.rows/2
                    self._assignedSearchMessage.waypoints.append(_wpMsg)
                    self.wpmsgQueued = True
                print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m ingress to search area:" + "\033[0m"                

    def _assign_ingress_UAVs_to_search(self):
        #print "\033[94m" + "Dylan test: SwarmSearcher _Assign any ingress UAV to start search" + "\033[0m"

        # loop through all ingress UAV: from swarm tracker
        # assign search_read uav to the cell
            #(Set altitude to SEARCH_ALTITUDE
            # set each UAV status and assignedTime "ross::Time::now()"
        reached = False
        currentTime = time.time()
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status == SEARCH_INGRESS and self.searchUAVMap[vehicle].subSwarmID == self.searchSubSwarmID:
                currentPose = self.searchUAVMap[vehicle].pose
                for j in range(self.rows):
                    for i in range(self.cols):
                        if reached == False:
                            tgt_cell = self.searchGrid[i][j].LLA
                            if gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1]) < SEARCH_ARRIVAL_THRESHOLD:
                                reached = True
                                self.searchUAVMap[vehicle].status = SEARCH_ACTIVE
                                self.searchUAVMap[vehicle].assignedCell = [i,j]
                                self.searchUAVMap[vehicle].assignedTime = currentTime
                                self.searchUAVMap[vehicle].assignedLLA = tgt_cell
                                self.searchUAVMap[vehicle].timeoutdistance = gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1])
                                self.searchUAVMap[vehicle].timeoutTime = currentTime
                                #print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m nears Search area" + "\033[0m"
                                print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m assigned entry via: " + "\033[93m["+str(i)+","+str(j)+"]\033[0m"
                                #assign UAV to tgt_cell
                                if vehicle == self.search_master_searcher_id:
                                    lla = apbrgmsg.LLA()
                                    lla.lat = tgt_cell[0]
                                    lla.lon = tgt_cell[1]
                                    lla.alt = self.searchUAVMap[vehicle].assignedAltitude
                                    self.publishWaypoint(lla)
                                else:
                                    _wpMsg = wpMsg()
                                    _wpMsg.recipientvehicle_id = vehicle
                                    _wpMsg.waypoint.lat = tgt_cell[0]
                                    _wpMsg.waypoint.lon = tgt_cell[1]
                                    _wpMsg.waypoint.alt = self.searchUAVMap[vehicle].assignedAltitude
                                    _wpMsg.searchCell_x = j
                                    _wpMsg.searchCell_y = i
                                    self._assignedSearchMessage.waypoints.append(_wpMsg)
                                    self.wpmsgQueued = True
        return reached

    def _check_search_operation(self):
        runOutofCells=False
        currentTime = time.time()
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status == SEARCH_ACTIVE and self.searchUAVMap[vehicle].subSwarmID == self.searchSubSwarmID:
                currentPose = self.searchUAVMap[vehicle].pose
                tgt_cell = self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].LLA
                if gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1]) < SEARCH_CELL_THRESHOLD:
                    if self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].visited == False:
                        print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[92m searched \033[94mcell \033[93m"+str(self.searchUAVMap[vehicle].assignedCell)+"\033[0m"
                        self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].visited = True
                        self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].visitedBy = vehicle
                        self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].visitedTimeStamp = time.time()
                    else:
                        print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m visited cell \033[93m"+str(self.searchUAVMap[vehicle].assignedCell)+"\033[0m"
                    result = self._assign_search_UAVs_to_nextCell(vehicle)
                    if result[0] == SEARCH_CELL_STILL_AVAILABLE: #assign UAV to tgt_cell
                        self.searchUAVMap[vehicle].assignedCell = [result[1],result[2]]
                        self.searchUAVMap[vehicle].assignedTime = currentTime
                        self.searchGrid[result[1]][result[2]].assigned = True
                        self.searchGrid[result[1]][result[2]].assignedTo = vehicle
                        self.searchUAVMap[vehicle].assignedLLA = self.searchGrid[result[1]][result[2]].LLA
                        self.searchUAVMap[vehicle].timeoutdistance = gps_distance(currentPose[0], currentPose[1], self.searchGrid[result[1]][result[2]].LLA[0], self.searchGrid[result[1]][result[2]].LLA[1])
                        self.searchUAVMap[vehicle].timeoutTime = currentTime
                        print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m assigned cell \033[93m"+str(self.searchUAVMap[vehicle].assignedCell)+"\033[0m"
                        if vehicle == self.search_master_searcher_id:
                            lla = apbrgmsg.LLA()
                            lla.lat = self.searchGrid[result[1]][result[2]].LLA[0]
                            lla.lon = self.searchGrid[result[1]][result[2]].LLA[1]
                            lla.alt = self.searchUAVMap[vehicle].assignedAltitude
                            self.publishWaypoint(lla)
                        else:
                            _wpMsg = wpMsg()
                            _wpMsg.recipientvehicle_id = vehicle
                            _wpMsg.waypoint.lat = self.searchGrid[result[1]][result[2]].LLA[0]
                            _wpMsg.waypoint.lon = self.searchGrid[result[1]][result[2]].LLA[1]
                            _wpMsg.waypoint.alt = self.searchUAVMap[vehicle].assignedAltitude
                            _wpMsg.searchCell_x = result[1]
                            _wpMsg.searchCell_y = result[2]
                            self._assignedSearchMessage.waypoints.append(_wpMsg)
                            self.wpmsgQueued = True
                    else:
                        if result[0] == SEARCH_CELL_FULLY_VISITED:
                            runOutofCells = True
                        self.searchUAVMap[vehicle].status = SEARCH_EGRESS
                        self.searchUAVMap[vehicle].assignedTime = currentTime
                        self.searchUAVMap[vehicle].assignedLLA = self.searchUAVMap[vehicle].originalLLA
                        self.searchUAVMap[vehicle].timeoutdistance = gps_distance(currentPose[0], currentPose[1], self.searchUAVMap[vehicle].originalLLA[0], self.searchUAVMap[vehicle].originalLLA[1])
                        self.searchUAVMap[vehicle].timeoutTime = currentTime
                        print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m returning home\033[0m"
                        if vehicle == self.search_master_searcher_id:
                            lla = apbrgmsg.LLA()
                            lla.lat = self.searchUAVMap[vehicle].originalLLA[0]
                            lla.lon = self.searchUAVMap[vehicle].originalLLA[1]
                            lla.alt = self.searchUAVMap[vehicle].assignedAltitude
                            self.publishWaypoint(lla)
                        else:
                            _wpMsg = wpMsg()
                            _wpMsg.recipientvehicle_id = vehicle
                            _wpMsg.waypoint.lat = self.searchUAVMap[vehicle].originalLLA[0]
                            _wpMsg.waypoint.lon = self.searchUAVMap[vehicle].originalLLA[1]
                            _wpMsg.waypoint.alt = self.searchUAVMap[vehicle].assignedAltitude
                            _wpMsg.searchCell_x = 254
                            _wpMsg.searchCell_y = 254
                            self._assignedSearchMessage.waypoints.append(_wpMsg)
                            self.wpmsgQueued = True
                #else: #Debug Prints... see how far the UAVs is from their destination
                    #print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m from assigned cell \033[93m"+str(self.searchUAVMap[vehicle].assignedCell)+"\033[94m  Distance: \033[0m" + str(gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1])) 
        return runOutofCells

    def _assign_search_UAVs_to_nextCell(self, vehicleID):
        # assign new unvisited cell if possible
        cellResult = SEARCH_CELL_FULLY_VISITED

        tgt_cell = [0,0]
        distance = SEARCH_MAX_THRESHOLD
        currentPose = self.searchUAVMap[vehicleID].pose
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
        # scan through all UAVs if any UAV did not get closer to weapon in the last SEARCH_TIMEOUT_INTERVAL seconds resend LLA
        currentTime = time.time()
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status in (SEARCH_INGRESS, SEARCH_ACTIVE, SEARCH_EGRESS): #any states that has assigned waypoints
                if self.searchUAVMap[vehicle].timeoutTime + SEARCH_TIMEOUT_INTERVAL < currentTime: # timeout interval up
                    distFromTgt = gps_distance(self.searchUAVMap[vehicle].pose[0], self.searchUAVMap[vehicle].pose[1], self.searchUAVMap[vehicle].assignedLLA[0], self.searchUAVMap[vehicle].assignedLLA[1])
                    if distFromTgt < self.searchUAVMap[vehicle].timeoutdistance: #uav is closer since last check. Update latest distance and time
                        if self.searchUAVMap[vehicle].status == SEARCH_EGRESS:
                            self.searchUAVMap[vehicle].timeoutdistance = distFromTgt + SEARCH_TIMEOUT_LOITERDISTANCEBUFFER
                        else:
                            self.searchUAVMap[vehicle].timeoutdistance = distFromTgt
                        self.searchUAVMap[vehicle].timeoutTime = currentTime
                        self.searchUAVMap[vehicle].timeoutCounter = 0
                    else:
                        if self.searchUAVMap[vehicle].timeoutCounter < SEARCH_TIMEOUT_STRIKES: #resend assigned lla
                            self.searchUAVMap[vehicle].timeoutCounter += 1
                            print "\033[94mSwarm Search node: \033[91mTIMEOUT "+str(self.searchUAVMap[vehicle].timeoutCounter)+"\033[94m for UAV \033[96m[" + str(vehicle) + "] \033[94mResend LLA... \033[0m"
                            self.searchUAVMap[vehicle].timeoutTime = currentTime
                            if vehicle == self.search_master_searcher_id:
                                lla = apbrgmsg.LLA()
                                lla.lat = self.searchUAVMap[vehicle].assignedLLA[0]
                                lla.lon = self.searchUAVMap[vehicle].assignedLLA[1]
                                lla.alt = self.searchUAVMap[vehicle].assignedAltitude
                                self.publishWaypoint(lla)
                            else:
                                _wpMsg = wpMsg()
                                _wpMsg.recipientvehicle_id = vehicle
                                _wpMsg.waypoint.lat = self.searchUAVMap[vehicle].assignedLLA[0]
                                _wpMsg.waypoint.lon = self.searchUAVMap[vehicle].assignedLLA[1]
                                _wpMsg.waypoint.alt = self.searchUAVMap[vehicle].assignedAltitude
                                _wpMsg.searchCell_x = self.searchUAVMap[vehicle].assignedCell[0]
                                _wpMsg.searchCell_y = self.searchUAVMap[vehicle].assignedCell[1]
                                self._assignedSearchMessage.waypoints.append(_wpMsg)
                                self.wpmsgQueued = True
                        else: # UAV not getting closer after all the timeout strikes, free assigned cell for others and place uav out of search operation
                            print "\033[94mSwarm Search node: UAV \033[96m[" + str(vehicle) + "] \033[91m STRIKEOUT! \033[0m"
                            if self.searchUAVMap[vehicle].status == SEARCH_ACTIVE:
                                if self._available_activeUAVHandover(vehicle) == True: # see if any other UAV active. if so, contiune so that they can take over the released cell
                                    self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].assigned=False
                                    self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].assignedTo=None
                                    print "\033[94mSearch cell \033[92m" + str(self.searchUAVMap[vehicle].assignedCell) + " \033[94m released for reassignment\033[0m"
                                else:
                                    egressUAVID = self._closest_egressUAVHandover(self.searchUAVMap[vehicle].assignedCell[0], self.searchUAVMap[vehicle].assignedCell[1])
                                    if egressUAVID != 0: #Found egress UAV to take over
                                        self.searchUAVMap[egressUAVID].status = SEARCH_ACTIVE
                                        self.searchUAVMap[egressUAVID].assignedCell = self.searchUAVMap[vehicle].assignedCell
                                        self.searchUAVMap[egressUAVID].assignedTime = currentTime
                                        self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].assigned = True
                                        self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].assignedTo = egressUAVID
                                        self.searchUAVMap[egressUAVID].assignedLLA = self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].LLA
                                        self.searchUAVMap[egressUAVID].timeoutdistance = gps_distance(self.searchUAVMap[egressUAVID].pose[0], self.searchUAVMap[egressUAVID].pose[1], self.searchUAVMap[egressUAVID].assignedLLA[0], self.searchUAVMap[egressUAVID].assignedLLA[1])
                                        self.searchUAVMap[egressUAVID].timeoutTime = currentTime
                                        print "\033[94m" + "Swarm Search node: Egressed Searcher \033[96m["+str(egressUAVID)+"]\033[94m assigned to take over strikedout \033[96m["+str(vehicle)+"]\033[94m assigned cell \033[93m"+str(self.searchUAVMap[vehicle].assignedCell)+"\033[0m"
                                        if egressUAVID == self.search_master_searcher_id:
                                            lla = apbrgmsg.LLA()
                                            lla.lat = self.searchUAVMap[egressUAVID].assignedLLA[0]
                                            lla.lon = self.searchUAVMap[egressUAVID].assignedLLA[1]
                                            lla.alt = self.searchUAVMap[egressUAVID].assignedAltitude
                                            self.publishWaypoint(lla)
                                        else:
                                            _wpMsg = wpMsg()
                                            _wpMsg.recipientvehicle_id = egressUAVID
                                            _wpMsg.waypoint.lat = self.searchUAVMap[egressUAVID].assignedLLA[0]
                                            _wpMsg.waypoint.lon = self.searchUAVMap[egressUAVID].assignedLLA[1]
                                            _wpMsg.waypoint.alt = self.searchUAVMap[egressUAVID].assignedAltitude
                                            _wpMsg.searchCell_x = self.searchUAVMap[egressUAVID].assignedCell[0]
                                            _wpMsg.searchCell_y = self.searchUAVMap[egressUAVID].assignedCell[1]
                                            self._assignedSearchMessage.waypoints.append(_wpMsg)
                                            self.wpmsgQueued = True
                                    else:
                                        print "\033[94mSearch \033[91m FAILED! \033[94m cell \033[92m" + str(self.searchUAVMap[vehicle].assignedCell) + " \033[94m released for assignment with no UAV to take over\033[0m"
                            self.searchUAVMap[vehicle].status = SEARCH_FAULT

    def _available_activeUAVHandover(self, faultyUAVID):
        available = False
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status in (SEARCH_INGRESS, SEARCH_ACTIVE):
                if vehicle != faultyUAVID:
                    available = True #Able to use "break" in python?
        return available

    def _closest_egressUAVHandover(self, cell_x, cell_y):
        closestUAV = 0
        closestDistance = SEARCH_MAX_THRESHOLD
        tgtLLA = self.searchGrid[cell_x][cell_y].LLA
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status == SEARCH_EGRESS:
                distFromTgt = gps_distance(self.searchUAVMap[vehicle].pose[0], self.searchUAVMap[vehicle].pose[1], tgtLLA[0], tgtLLA[0])
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
        for vehicle in swarmMsg.swarm:
            if vehicle.vehicle_id in self.searchUAVMap:
                self.searchUAVMap[vehicle.vehicle_id].pose = [vehicle.state.pose.pose.position.lat, vehicle.state.pose.pose.position.lon]
                self.searchUAVMap[vehicle.vehicle_id].subSwarmID = vehicle.subswarm_id
                if vehicle.swarm_state != self.searchUAVMap[vehicle.vehicle_id].receivedState:
                    if self.ExecuteMasterSearcherBehavior == True:
                        if self.searchUAVMap[vehicle.vehicle_id].receivedState == SEARCH_SWARM_SEARCH_ACTIVE_STATE and vehicle.swarm_state == SEARCH_SWARM_SEARCH_READY_STATE:
                            print "\033[92mUAV \033[96m[" + str(vehicle.vehicle_id) + "] \033[94mterminated swarm behaviour. \033[0m"
                            self.searchUAVMap[vehicle.vehicle_id].status = SEARCH_FAULT
                            if self.ownID == vehicle.vehicle_id:
                                currentTime = time.time()
                                timeDelta = currentTime - self.search_Operation_StartTime
                                print "\033[94mSearch Operation \033[91mSUSPENDED \033[94mafter \033[96m"+"{0:.3f}".format(timeDelta)+"\033[94m secs\033[0m"
                                for j in range(self.rows):
                                    for i in range(self.cols):
                                        if self.searchGrid[i][j].visited == False:
                                            print "\033[93m [" + str(i) + "," + str(j) + "] \033[94mwas not visited.\033[0m"
                                        else:
                                            timeDelta = self.searchGrid[i][j].visitedTimeStamp - self.search_Run_StartTime
                                            print "\033[93m [" + str(i) + "," + str(j) + "] \033[94mVisit by \033[96m"+ str(self.searchGrid[i][j].visitedBy) + "\033[94m after \033[96m"+"{0:.1f}".format(timeDelta)+"\033[94m secs\033[0m"
                                self.SEARCH_STATUS = SEARCH_READY
                                self.searchGrid = None
                                self.ExecuteMasterSearcherBehavior = False
                                self._searchDeactivatePublisher.publish(1)
                    self.searchUAVMap[vehicle.vehicle_id].receivedState = vehicle.swarm_state
                    if self.searchUAVMap[vehicle.vehicle_id].receivedState == SEARCH_SWARM_SEARCH_ACTIVE_STATE:
                        self.searchUAVMap[vehicle.vehicle_id].status = SEARCH_READY # This is correct. When swarm behaviour just turn active, the first local state is search ready
            else:
                self.searchUAVMap[vehicle.vehicle_id] = searchUAVdata()
                self.searchUAVMap[vehicle.vehicle_id].pose = [vehicle.state.pose.pose.position.lat, vehicle.state.pose.pose.position.lon]
                self.searchUAVMap[vehicle.vehicle_id].subSwarmID = vehicle.subswarm_id
                self.searchUAVMap[vehicle.vehicle_id].assignedAltitude = SEARCH_ALTITUDE + (self.uavAltitudeCounter * SEARCH_SAFETY_ALTITUDE_INTERVAL)
                self.uavAltitudeCounter += 1

    def _process_swarm_search_waypoint(self, swarmSearchWP):
        # Check if own UAV supposed to recepient of this message
        for waypointMsg in swarmSearchWP.waypoints:
            if self.ownID == waypointMsg.recipientvehicle_id:
                if waypointMsg.searchCell_x >= 254 and waypointMsg.searchCell_y >= 254:
                    self._searchDeactivatePublisher.publish(1)
                    print "\033[94m" + "Swarm Searcher \033[96m["+str(self.ownID)+"]\033[94m proceeding to deactivate\033[0m"
                else:
                    lla = apbrgmsg.LLA()
                    lla.lat = waypointMsg.waypoint.lat
                    lla.lon = waypointMsg.waypoint.lon
                    lla.alt = self.searchUAVMap[self.ownID].assignedAltitude
                    self.publishWaypoint(lla)
                    print "\033[94m" + "Swarm Searcher \033[96m["+str(self.ownID)+"]\033[94m proceeding to assigned cell \033[93m[" + str(waypointMsg.searchCell_x)+", "+ str(waypointMsg.searchCell_y)+"]\033[0m"
            else:
                if waypointMsg.searchCell_x >= 254 and waypointMsg.searchCell_y >= 254:
                    print "\033[94m" + "Swarm Search \033[96m["+str(self.ownID)+"]\033[94m node: Received network wp cmd for Slave Searcher \033[96m["+str(waypointMsg.recipientvehicle_id)+"]\033[94m to deactivate\033[0m"

    def _process_swarmSearch_setup(self, swarmSearchSetup):
        try:
            # Future to-do:
            # Check subswarmID that this setup message is for

            if self.rel_Altitude is None: raise ValueError('Relative Altitude has not been specified yet')
            self.search_master_searcher_id = swarmSearchSetup.order.masterSearcherID
            if self.ownID == self.search_master_searcher_id:
                print "\033[94mSwarm Search Master \033[96m["+str(self.ownID)+"]\033[94m received command from swarm manager \033[96m"+str(swarmSearchSetup)+"\033[0m"
                if self.ExecuteMasterSearcherBehavior == False:
                    self.search_Operation_StartTime = time.time()
                    self.bottomLeftLLA = [swarmSearchSetup.order.lat, swarmSearchSetup.order.lon]
                    self.selectedSearchAlgo = 1 # Only 1 is available now
                    tempVar = swarmSearchSetup.order.searchAreaLength / SEARCH_CELL_RADIUS
                    self.cols = int(math.ceil(tempVar))
                    tempVar = swarmSearchSetup.order.searchAreaWidth / SEARCH_CELL_RADIUS
                    self.rows = int(math.ceil(tempVar))
                    self.numOfRunsToDo = 1
                    self.numOfRunsDone = 0
                    self.searchSubSwarmID = self.searchUAVMap[self.ownID].subSwarmID
                    self.ExecuteMasterSearcherBehavior = True
                    self.SEARCH_STATUS = SEARCH_READY
                    self.searchGrid = None
                    print "\033[94mSwarm Search Master \033[96m["+str(self.ownID)+"]\033[94m search operation with \033[96m"+ str(self.numOfRunsToDo) +"\033[94m run(s) started on \033[96m"+str(time.asctime(time.localtime(time.time())))+"\033[0m"
                    for vehicle in self.searchUAVMap:
                        self.searchUAVMap[vehicle].assignedAltitude = self.rel_Altitude
                    print "\033[94mSwarm Search Master \033[96m[" + str(self.ownID) + "] \033[94massigned relative altitude for search \033[0m" + str(self.searchUAVMap[self.ownID].assignedAltitude)
                    self._activate_swarm_search()
                    self.set_ready_state(True)
                else:
                    print "\033[94mSwarm Search Master \033[96m["+str(self.ownID)+"]\033[94m ignored command from swarm manager as search operation is underway. Suspend operation first before assigning any new operation\033[0m"
            else:
                self.ExecuteMasterSearcherBehavior = False
                self.searchUAVMap[self.ownID].assignedAltitude = self.rel_Altitude
                print "\033[94mSwarm Search node: Searcher \033[96m[" + str(self.ownID) + "] \033[94massigned relative altitude for search \033[0m" + str(self.searchUAVMap[self.ownID].assignedAltitude)
            self.set_ready_state(True)
        except Exception as ex:
            self.log_warn("Failed to initialize swarm search: " + str(ex))
            self.set_ready_state(False)

    def sub_swarm_relAlt_update(self, msg):
        self.rel_Altitude = msg.pose.pose.position.rel_alt
#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    searcher = SwarmSearcher("swarm_searcher",rospy.get_param("aircraft_id"),args)
    searcher.runAsNode(10, [], [], [])
    #print "\033[94m" + "SwarmSearcher main code, args is: " + str(args) + "\033[0m"
