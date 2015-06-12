#!/usr/bin/env python

# Swarm_searcher code implemented as a node.
# Improved version written by Stefan Jorgensen, August 2014
#
# Modified by Duane Davis (object implementation)
# Modified by Dylan Lau (Swarm_searcher impementation as object)

import sys
import rospy
import time
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from argparse import ArgumentParser

# Import ROS message and service types
import std_msgs.msg as stdmsg


from ap_lib.gps_utils import *
from ap_lib import nodeable
from ap_lib.controller import *
from ap_lib.waypoint_controller import *

# Base name for node topics and services
NODENAME = 'swarm_searcher'

# Global variables (constants)
# Enumeration for swarming search states
SEARCH_SLAVE = 0   # All search slaves
SEARCH_STAGING = 1 # Preparing for search
SEARCH_READY = 2   # Awaiting search area
SEARCH_INGRESS = 3 # Received search area and transiting to it
SEARCH_ACTIVE = 4  # Searching in search area
SEARCH_EGRESS = 5  # search end/completed and transiting to staging area
SEARCH_TRACKING = 6# search uav detected something is tasked to track it

SEARCH_CELL_FULLY_VISITED = 0 # All cells visited
SEARCH_CELL_FULLY_ASSIGNED = 1 # All cells assigned
SEARCH_CELL_STILL_AVAILABLE = 3 # Cells still available for assignment

SEARCH_CELL_RADIUS = 150 # Interval between search cell in metres
SEARCH_ALTITUDE = 50 # Height to conduct search in metres
SEARCH_SAFETY_ALTITUDE_INTERVAL = 15 # Height between UAV of the same cell in metres
SEARCH_CELL_THRESHOLD = 75 # Distance to determine that cell is searched (must be greater than infinite loiter radius
SEARCH_ARRIVAL_THRESHOLD = 150 #
SEARCH_MAX_THRESHOLD = 100000000 # Max number for comparsion

SEARCH_SWARM_READY_STATE = 2 # enum when swarm completed staging and is ready for search

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
    receivedState=None
    pose=None
    assignedCell=None
    assignedTime=None
    originalLLA=None
# Class member variables:
#
# Inherited from Nodeable:
#   nodeName:  Name of the node to start or node in which the object is
#   timer: ROS rate object that controls the timing loop
#   DBUG_PRINT: set true to force screen debug messages (default FALSE)
#   WARN_PRINT: set false to force screen warning messages (default FALSE)
#
# Class member functions:
#class SwarmSearcher(nodeable.Nodeable):
class SwarmSearcher(WaypointController):


    search_master_searcher_id = None #variable for master searcher id
    #2D Array to store the 'cell' objects
    searchGrid = None
    # Search Swarm UAV status
    searchUAVMap = dict()

    # Assume Search Grid message states BottomLeft LLA plus length and width
    bottomLeftLLA = [35.718788,-120.768071]

    centerofSearchGridLLA = None

    # assume length and width divided by SEARCH_CELL_RADIUS (150 metres) equals cols and rows
    # may need to reduce to keep bigger buffer btw fence and search area
    cols = 5
    rows = 4

    # Fake counter to trigger staging completed
    counterA = 30
    # Fake counter to trigger search area received
    counterB = 10

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
        WaypointController.__init__(self, nodename, 0) #Assume this controller ID is 0
        self.DBUG_PRINT = False
        self.WARN_PRINT = False
        # Boolean flag to determine Centralised search node
        self.SEARCH_MASTER = False
        self.SEARCH_STATUS = SEARCH_SLAVE
        self.ownID = ownAC

        self._swarmSearchPublisher = None
        self._assignedSearchMessage = apmsg.SwarmSearchWaypoint()

        self.search_master_searcher_id = int(args[1]) # master searcher ID

        # use the args in the launch file toHardcode 101 as the search master and the rest as search slaves
        if self.ownID == self.search_master_searcher_id:
           self.SEARCH_MASTER = True
           self.SEARCH_STATUS = SEARCH_STAGING
           print "\033[94m" + "Swarm Search node init: \033[96m"+ str(ownAC) + "\033[94m is search master " + "\033[0m"
        else:
           self.SEARCH_MASTER = False
           print "\033[94m" + "Swarm Search node init: \033[96m"+ str(ownAC) + "\033[94m is search slave " + "\033[0m"


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
        self.createSubscriber("recv_swarm_search_waypoint", apmsg.SwarmSearchWaypoint, \
                                self._process_swarm_search_waypoint)
    def publisherSetup(self, params=[]):
        self._swarmSearchPublisher = \
            self.createPublisher("send_swarm_search_waypoint", apmsg.SwarmSearchWaypoint, 1)

    def serviceSetup(self, params=[]):
        pass

    def executeTimedLoop(self):
        if self.SEARCH_MASTER == True:
            #for vehicle in self.searchUAVMap:
            #    print "\033[94m TimeLoop: UAV \033[96m[" + str(vehicle) + "] \033[94m Received State ["+str(self.searchUAVMap[vehicle].receivedState)+"] pose: \033[0m" + str(self.searchUAVMap[vehicle].pose)
                
            if self.SEARCH_STATUS == SEARCH_STAGING:
                if self.search_master_searcher_id in self.searchUAVMap:
                    if self.searchUAVMap[self.search_master_searcher_id].receivedState == SEARCH_SWARM_READY_STATE:
                        self.SEARCH_STATUS = SEARCH_READY
                        print "\033[94m" + "Swarm Search node: search swarm ready" + "\033[0m"
            elif self.SEARCH_STATUS == SEARCH_READY:
                print "\033[94m" + "Swarm Search node: receiving search area count down " + "\033[0m" + str(self.counterB)
                self.counterB -= 1
                if self.counterB < 1:
                    self.SEARCH_STATUS = SEARCH_INGRESS
                    self._activate_swarm_search()
                    self._assign_ready_UAVs_to_searchGrid()
                    self.counterB = 10 # to repeat search ops
            elif self.SEARCH_STATUS == SEARCH_INGRESS:
                self._assign_ready_UAVs_to_searchGrid()

                # Check if any of the search uav has reached the search area. set search status to active if true
                reached = self._assign_ingress_UAVs_to_search()

                if reached == True:
                    print "\033[94m" + "Swarm Search node: search started" + "\033[0m"
                    self.SEARCH_STATUS = SEARCH_ACTIVE
            elif self.SEARCH_STATUS == SEARCH_ACTIVE:
                self._assign_ready_UAVs_to_searchGrid()
                self._assign_ingress_UAVs_to_search()

                # Check if search is completed (all cells visited) set status to SEARCH_EGRESS
                searchComplete = self._check_search_operation()

                if searchComplete == True:
                    print "\033[94m" + "Swarm Search node: search completed" + "\033[0m"
                    for j in range(self.rows):
                        for i in range(self.cols):
                            print "\033[93m [" + str(i) + "," + str(j) + "] \033[94m Visit By: \033[96m"+ str(self.searchGrid[i][j].visitedBy) + " \033[94m at Timestamp: \033[0m" + str(self.searchGrid[i][j].visitedTimeStamp)


                    self.SEARCH_STATUS = SEARCH_EGRESS
                    # set all ingress/active uav to egress
                    for vehicle in self.searchUAVMap:
                        if self.searchUAVMap[vehicle].status == SEARCH_INGRESS or self.searchUAVMap[vehicle].status == SEARCH_ACTIVE:
                            self.searchUAVMap[vehicle].status = SEARCH_EGRESS
                            self.searchUAVMap[vehicle].assignedTime = time.time()
                            lla = apbrg.LLA()
                            lla.lat = self.searchUAVMap[vehicle].originalLLA[0]
                            lla.lon = self.searchUAVMap[vehicle].originalLLA[1]
                            lla.alt = SEARCH_ALTITUDE
                            if vehicle == self.search_master_searcher_id:
                                self.publishWaypoint(lla)

            elif self.SEARCH_STATUS == SEARCH_EGRESS:
                # check if all search slaves has reach originalLLA and set status to search ready
                # and clear search grid
                egressComplete = True
                for vehicle in self.searchUAVMap:
                    if self.searchUAVMap[vehicle].status == SEARCH_EGRESS:
                        egressComplete = False
                        currentPose = self.searchUAVMap[vehicle].pose
                        tgt_cell = self.searchUAVMap[vehicle].originalLLA
                        if gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1]) < SEARCH_ARRIVAL_THRESHOLD:
                            self.searchUAVMap[vehicle].status = SEARCH_READY

                if egressComplete == True:
                    print "\033[94m" + "Swarm Search node: Egress completed, ready for another search" + "\033[0m"
                    self.SEARCH_STATUS = SEARCH_READY
                    self.searchGrid = None

    #--------------------------
    # Object-specific functions
    #--------------------------
    def _activate_swarm_search(self):
        print "\033[94m" + "Swarm Search node: Generate Search Grid:" + "\033[0m"

        # Create the searchGrid
        self.searchGrid = [[cell() for j in range(self.rows)] for i in range(self.cols)]

        # populate the searchGrid
        for j in range(self.rows):
            for i in range(self.cols):
                self.searchGrid[i][j].grid = [i * SEARCH_CELL_RADIUS, j * SEARCH_CELL_RADIUS]
                self.searchGrid[i][j].LLA = gps_offset(self.bottomLeftLLA[0], self.bottomLeftLLA[1], i * SEARCH_CELL_RADIUS, j * SEARCH_CELL_RADIUS)
                print "\033[93m [" + str(i) + "," + str(j) + "] \033[36m Grid: "+ str(self.searchGrid[i][j].grid) + " \033[0mLLA: " + str(self.searchGrid[i][j].LLA)

        # Assign ownself to center of search grid
        self.centerofSearchGridLLA = self.searchGrid[self.cols/2][self.rows/2].LLA
        lla = apbrg.LLA()
        lla.lat = self.centerofSearchGridLLA[0]
        lla.lon = self.centerofSearchGridLLA[1]
        lla.alt = SEARCH_ALTITUDE
        print "\033[94m" + "Swarm Search node: Master searcher ingress to Search area:" + "\033[0m"
        self.publishWaypoint(lla)
        self.searchUAVMap[self.search_master_searcher_id].status = SEARCH_INGRESS
        self.searchUAVMap[self.search_master_searcher_id].originalLLA = self.searchUAVMap[self.search_master_searcher_id].pose

    def _assign_ready_UAVs_to_searchGrid(self):
        #print "\033[94m" + "Dylan test: SwarmSearcher _Assign any ready UAV to search grid" + "\033[0m"

        # assign all "search_ready" uav to the nearest cell in the search area
        # loop through all ready UAV: from swarm tracker
        # assign search_read uav to center of search grid
        # set each UAV status
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status == SEARCH_READY:
                self.searchUAVMap[vehicle].status = SEARCH_INGRESS
                self.searchUAVMap[vehicle].originalLLA = self.searchUAVMap[vehicle].pose

                self._assignedSearchMessage.recipientvehicle_id = vehicle
                self._assignedSearchMessage.waypoint.lat = self.centerofSearchGridLLA[0]
                self._assignedSearchMessage.waypoint.lon = self.centerofSearchGridLLA[1]
                self._assignedSearchMessage.waypoint.alt = SEARCH_ALTITUDE
                self._assignedSearchMessage.searchCell_x = self.cols/2
                self._assignedSearchMessage.searchCell_y = self.rows/2
                self._swarmSearchPublisher.publish(self._assignedSearchMessage)
                print "\033[94m" + "Swarm Search node: Slave searcher \033[96m["+str(vehicle)+"]\033[94m ingress to Search area:" + "\033[0m"

    def _assign_ingress_UAVs_to_search(self):
        #print "\033[94m" + "Dylan test: SwarmSearcher _Assign any ingress UAV to start search" + "\033[0m"

        # loop through all ingress UAV: from swarm tracker
        # assign search_read uav to the cell
            #(Set altitude to SEARCH_ALTITUDE
            # set each UAV status and assignedTime "ross::Time::now()"
        reached = False
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status == SEARCH_INGRESS:
                currentPose = self.searchUAVMap[vehicle].pose
                for j in range(self.rows):
                    for i in range(self.cols):
                        if reached == False:
                            tgt_cell = self.searchGrid[i][j].LLA
                            if gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1]) < SEARCH_ARRIVAL_THRESHOLD:
                                reached = True
                                self.searchUAVMap[vehicle].status = SEARCH_ACTIVE
                                self.searchUAVMap[vehicle].assignedCell = [i,j]
                                self.searchUAVMap[vehicle].assignedTime = time.time()
                                print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m nears Search area" + "\033[0m"
                                print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m assigned entry via: " + "\033[93m["+str(i)+","+str(j)+"]\033[0m"
                                #assign UAV to tgt_cell
                                if vehicle == self.search_master_searcher_id:
                                    lla = apbrg.LLA()
                                    lla.lat = tgt_cell[0]
                                    lla.lon = tgt_cell[1]
                                    lla.alt = SEARCH_ALTITUDE
                                    self.publishWaypoint(lla)
                                else:
                                    self._assignedSearchMessage.recipientvehicle_id = vehicle
                                    self._assignedSearchMessage.waypoint.lat = tgt_cell[0]
                                    self._assignedSearchMessage.waypoint.lon = tgt_cell[1]
                                    self._assignedSearchMessage.searchCell_x = j
                                    self._assignedSearchMessage.searchCell_y = i
                                    self._swarmSearchPublisher.publish(self._assignedSearchMessage)                
        return reached

    def _check_search_operation(self):
        runOutofCells=False
        for vehicle in self.searchUAVMap:
            if self.searchUAVMap[vehicle].status == SEARCH_ACTIVE:
                currentPose = self.searchUAVMap[vehicle].pose
                tgt_cell = self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].LLA
                if gps_distance(currentPose[0], currentPose[1], tgt_cell[0], tgt_cell[1]) < SEARCH_CELL_THRESHOLD:
                    if self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].visited == False:
                        print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m searched cell \033[93m"+str(self.searchUAVMap[vehicle].assignedCell)+"\033[0m"
                        self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].visited = True
                        self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].visitedBy = vehicle
                        self.searchGrid[self.searchUAVMap[vehicle].assignedCell[0]][self.searchUAVMap[vehicle].assignedCell[1]].visitedTimeStamp = time.time()
                    else:
                        print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m visited cell \033[93m"+str(self.searchUAVMap[vehicle].assignedCell)+"\033[0m"
                    result = self._assign_search_UAVs_to_nextCell(vehicle)
                    if result[0] == SEARCH_CELL_STILL_AVAILABLE:
                        self.searchUAVMap[vehicle].assignedCell = [result[1],result[2]]
                        self.searchUAVMap[vehicle].assignedTime = time.time()
                        self.searchGrid[result[1]][result[2]].assigned = True
                        self.searchGrid[result[1]][result[2]].assignedTo = vehicle
                        #assign UAV to tgt_cell
                        print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m assigned cell \033[93m"+str(self.searchUAVMap[vehicle].assignedCell)+"\033[0m"
                        if vehicle == self.search_master_searcher_id:
                            lla = apbrg.LLA()
                            lla.lat = self.searchGrid[result[1]][result[2]].LLA[0]
                            lla.lon = self.searchGrid[result[1]][result[2]].LLA[1]
                            lla.alt = SEARCH_ALTITUDE
                            self.publishWaypoint(lla)
                        else:
                            self._assignedSearchMessage.recipientvehicle_id = vehicle
                            self._assignedSearchMessage.waypoint.lat = self.searchGrid[result[1]][result[2]].LLA[0]
                            self._assignedSearchMessage.waypoint.lon = self.searchGrid[result[1]][result[2]].LLA[1]
                            self._assignedSearchMessage.waypoint.alt = SEARCH_ALTITUDE
                            self._assignedSearchMessage.searchCell_x = result[1]
                            self._assignedSearchMessage.searchCell_y = result[2]
                            self._swarmSearchPublisher.publish(self._assignedSearchMessage)
                    else:
                        if result[0] == SEARCH_CELL_FULLY_VISITED:
                            runOutofCells = True
                        self.searchUAVMap[vehicle].status = SEARCH_EGRESS
                        self.searchUAVMap[vehicle].assignedTime = time.time()
                        print "\033[94m" + "Swarm Search node: Searcher \033[96m["+str(vehicle)+"]\033[94m returning home\033[0m"
                        if vehicle == self.search_master_searcher_id:
                            lla = apbrg.LLA()
                            lla.lat = self.searchUAVMap[vehicle].originalLLA[0]
                            lla.lon = self.searchUAVMap[vehicle].originalLLA[1]
                            lla.alt = SEARCH_ALTITUDE
                            self.publishWaypoint(lla)
                        else:
                            self._assignedSearchMessage.recipientvehicle_id = vehicle
                            self._assignedSearchMessage.waypoint.lat = self.searchUAVMap[vehicle].originalLLA[0]
                            self._assignedSearchMessage.waypoint.lon = self.searchUAVMap[vehicle].originalLLA[1]
                            self._assignedSearchMessage.waypoint.alt = SEARCH_ALTITUDE
                            self._assignedSearchMessage.searchCell_x = 255
                            self._assignedSearchMessage.searchCell_y = 255
                            self._swarmSearchPublisher.publish(self._assignedSearchMessage)
                #else:
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

    #------------------------------------------
    # ROS Subscriber callbacks -for this object
    #------------------------------------------
    # Handle incoming swarm_uav_states messages
    # @param swarmMsg: message containing swarm data (SwarmStateStamped)
    def _process_swarm_uav_states(self, swarmMsg):
        if self.ownID == self.search_master_searcher_id:
            for vehicle in swarmMsg.swarm:
                if vehicle.vehicle_id in self.searchUAVMap:
                    self.searchUAVMap[vehicle.vehicle_id].pose = [vehicle.state.pose.pose.position.lat, vehicle.state.pose.pose.position.lon]
                    self.searchUAVMap[vehicle.vehicle_id].receivedState = vehicle.swarm_state
                    #print "\033[94m updated new UAV [" + str(vehicle.state.pose.pose.position) + " \033[0m"
                    #print "\033[94m updated UAV \033[96m[" + str(vehicle.vehicle_id) + "] \033[0m pos"
                    #print "\033[94m UAV \033[96m[" + str(vehicle.vehicle_id) + "] \033[94m ["+str(vehicle.swarm_state)+", "+str(vehicle.swarm_behavior)+"]\033[0m"
                else:
                    print "\033[94mFound \033[92mnew UAV \033[96m[" + str(vehicle.vehicle_id) + "] \033[0m"
                    #print "\033[94m with pose [" + str(vehicle.state.pose.pose.position) + " \033[0m"
                    self.searchUAVMap[vehicle.vehicle_id] = searchUAVdata()
                    self.searchUAVMap[vehicle.vehicle_id].pose = [vehicle.state.pose.pose.position.lat, vehicle.state.pose.pose.position.lon]

                if self.searchUAVMap[vehicle.vehicle_id].receivedState == SEARCH_SWARM_READY_STATE:
                    if self.searchUAVMap[vehicle.vehicle_id].status == None:
                        self.searchUAVMap[vehicle.vehicle_id].status = SEARCH_READY
                        print "\033[92mUAV \033[96m[" + str(vehicle.vehicle_id) + "] \033[94mready for search \033[0m"

    def _process_swarm_search_waypoint(self, swarmSearchWP):
        # Check if own UAV supposed to recepient of this message
        print "\033[94m" + "Swarm Search node: Received network wp cmd for Slave Searcher \033[96m["+str(swarmSearchWP.recipientvehicle_id)+"]\033[94m to visit assigned cell \033[93m[" + str(swarmSearchWP.searchCell_x)+", "+ str(swarmSearchWP.searchCell_y)+"]\033[0m"
        if self.ownID == swarmSearchWP.recipientvehicle_id:
            lla = apbrg.LLA()
            lla.lat = swarmSearchWP.waypoint.lat
            lla.lon = swarmSearchWP.waypoint.lon
            lla.alt = swarmSearchWP.waypoint.alt
            self.publishWaypoint(lla)
            print "\033[94m" + "Swarm Search node: Slave Searcher \033[96m["+str(self.ownID)+"]\033[94m proceeding to assigned waypoint at \033[93m"+str(lla)+"\033[0m"

#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    searcher = SwarmSearcher("swarm_searcher",rospy.get_param("aircraft_id"),args)
    searcher.runAsNode(1, [], [], [])
    #print "\033[94m" + "SwarmSearcher main code, args is: " + str(args) + "\033[0m"
