#!/usr/bin/env python

#-----------------------------------------------------------------------
# Multiple waypoint sequencer
# Duane Davis, 2014
#
# Generates sequences of waaypoints and issues them to the autopilot in
# order based on the waypoint sequence and vehicle location
#-----------------------------------------------------------------------

#----------------------------
# Import a bunch of libraries
#----------------------------

# Standard Python imports
from collections import deque

# General ROS imports
import rospy

# Imports from other ACS packages and code sources
from MAVProxy.modules.lib import mp_util


# Import ROS message and service types
import std_msgs.msg as stdmsg
from geometry_msgs.msg import PoseWithCovarianceStamped
from autopilot_bridge.msg import LLA
import ap_path_planning.msg as appp

#import std_srvs.srv

# Base name for node topics and services
NODE_BASENAME = 'wp_sequencer'
ODOM_BASENAME = 'local_estim'
AP_BASENAME = 'autopilot'

# Control printing of messages to stdout
DBUG_PRINT = False
WARN_PRINT = False

# Other global constants
CAPTURE_DISTANCE = 100.0


# Object that creates or receives waypoint sequences and monitors the
# vehicle's progress through the series of waypoints
#
# Class member variables:
#   nodeName: name of the ROS node with which this object is associated
#   wpList: list (queue) of waypoints being followed
#   currentWP: waypoint to which the vehicle is currently transiting
#   pose: current vehicle position
#   wpPublisher: publisher object for publishing waypoint commands
#   captureDistance: distance from waypoint considered captured
#   listComplete: set to true when the last waypoint is reached
#   isRunning: set to True is the waypoint sequencer is running
#   readyNextWP: set to True when the next waypoint can be issued
class WaypointSequencer(object):


    # Class initializer initializes variables, subscribes to required
    # ROS topics, and creates required ros publishers.  Initializer
    # assumes that the object is already running within an initialized
    # ROS node (i.e., the object does not initialize itself as a node).
    # This enables multiple objects to run within a single node if
    # required.
    # @param waypoints: list of waypoints in the desired traversal order
    # @param nodeName: name of the ROS node for this object
    # @param odomBaseName: base name of ROS topic to read odometry from
    # @param apBaseName: base name of the ROS topic to publish waypoints to
    # @param captureDistance: distance from a waypoint considered good enough
    def __init__(self, waypoints=[], nodeName=NODE_BASENAME,\
                 odomBaseName=ODOM_BASENAME, apBaseName=AP_BASENAME,\
                 captureDistance=CAPTURE_DISTANCE):
        self.nodeName = nodeName
        self.currentWP = LLA()
        self.pose = None
        self.setSequence(waypoints)
        self.isRunning = False
        self.captureDistance = captureDistance

        # Set up object-specific ROS publishers
        self.wpPublisher = rospy.Publisher("%s/payload_waypoint"%apBaseName, LLA)

        # Set up object-specific ROS subscribers
        rospy.Subscriber("%s/odom_combined"%odomBaseName, \
                         PoseWithCovarianceStamped, self.updatePose)
        rospy.Subscriber("%s/wp_sequencer_run"%nodeName, stdmsg.Bool, \
                         self.processRunMsg)
        rospy.Subscriber("%s/wp_list"%nodeName, appp.WaypointListStamped, \
                         self.receiveWaypointList)

        # Set up object-specific ROS services


    #--------------------------
    # Object-specific functions
    #--------------------------

    # Updates the waypoint sequence.  Any existing waypoint sequence
    # will be deleted, and the new sequence will be immediately 
    # commenced.  If the new sequence contains no waypoints, any
    # existing sequence will be deleted, but any current sequenced
    # waypoint that was previously commanded will remain in effect
    # newWaypoints: list of waypoints in the desired traversal order
    def setSequence(self, newWaypoints):
        try:
            self.wpList = deque(newWaypoints)
            if len(self.wpList) > 0:
                self.readyNextWP = True
                self.listComplete = False
            else:
                self.readyNextWP = False
                self.listComplete = True
            self.log_dbug("set waypoint sequence: " + str(newWaypoints))
        except Exception as exc:
            self.log_warn("exception resetting waypoint sequence: " + \
                          str(newWaypoints))
            self.wpList = deque()
            self.listComplete = True  # If there's been an error--quit!


    # Adds a single waypoint to the end of the current list
    # @param newWaypoint:  the waypoint to be added to the list
    def addWaypoint(self, newWaypoint):
        self.wpList.append(newWaypoint)
        self.listComplete = False


    # Pops the first sequence from the waypoint list and updates
    # the currently commanded waypoint with the new value
    def incrementWP(self):
        self.readyNextWP = False
        if (len(self.wpList) > 0):
            nextWP = self.wpList.popleft()
            self.currentWP.lat = nextWP[0]
            self.currentWP.lon = nextWP[1]
            self.currentWP.alt = nextWP[2]
            self.wpPublisher.publish(self.currentWP)
            self.log_dbug("issuing new wpt: lat=" + str(self.currentWP.lat) +\
                                         ", lon=" + str(self.currentWP.lon) +\
                                         ", alt=" + str(self.currentWP.alt))
        else:
            self.log_dbug("reached last waypoint in sequence")
            self.listComplete = True
            

    # Checks to see if it is time to issue the next waypoint
    def checkReadyNextWP(self):
        try:
            if self.listComplete:  return False
            if self.readyNextWP:  return True
            d = mp_util.gps_distance(self.pose[0], self.pose[1], \
                                     self.currentWP.lat, self.currentWP.lon)
            if d < self.captureDistance:
                self.readyNextWP = True
                self.log_dbug("waypoint reached: lat=" + str(self.currentWP.lat) +\
                                              ", lon=" + str(self.currentWP.lon) +\
                                              ", alt=" + str(self.currentWP.alt))
            return self.readyNextWP
        except Exception as exc:
            self.log_warn("exception checking waypoint distance")
            print exc
            self.readyNextWP = False
            self.listComplete = True
            return False


    # Starts or stops the waypoint sequencer object.  At instantiation,
    # the WaypointSequencer isRunning variable is set to False (no waypoints
    # issued).  It can be set to True or False using this method)
    # @param newRunState:  True or False to start or stop object run state
    def startSequencer(self, newRunState):
        self.isRunning = newRunState
    

    # Runs one iteration of the sequencer checks to see if the current
    # waypoint has been reached, and if so, issues the next one
    def loopOnce(self):
        if (self.isRunning and not self.listComplete):
            self.checkReadyNextWP()
            if self.readyNextWP:  self.incrementWP()

            
    #-----------------------------------------
    # ROS Subscriber callbacks for this object
    #-----------------------------------------

    # Handles new pose information for this aircraft so that the vehicle
    # can determine where it is relative to the current waypoint
    # @param poseMsg: PoseWithCovarianceStamped object with the new pose
    def updatePose(self, poseMsg):
        self.pose = [ poseMsg.pose.pose.position.x, \
                      poseMsg.pose.pose.position.y ]


    # Handles start and stop commands written to the ROS topic
    # @param runMsg: ROS message (std_msgs/Bool) with the start/stop command
    def processRunMsg(self, runMsg):
        self.startSequencer(runMsg.data)
        self.log_dbug("received start/stop message=" + str(runMsg.data))


    # Handles waypoint lists written to the ROS topic
    # @param wptMsg: ROS message (WaypointListStamped) with the waypoint list
    def receiveWaypointList(self, wpMsg):
        wpts = []
        for wpt in wptList.waypoints:
           wpts.append([ wpt.lat, wpt.lon, wpt.alt ])
        setSequence(wpts)


    #-----------------------------
    # ROS services for this object
    #-----------------------------


    #--------------------------------------
    # Logging and message utility functions
    #--------------------------------------

    # Logging utility function for this object
    def log_dbug(self, msg):
        rospy.logdebug(msg)
        if DBUG_PRINT:
            print "..DEBUG.. %s: %s" % (self.nodeName, msg)


    # Warning utility function for this object
    def log_warn(self, msg):
        rospy.logwarn(msg)
        if WARN_PRINT:
            print "**WARN** %s: %s" % (self.nodeName, msg)

