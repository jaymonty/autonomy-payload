#!/usr/bin/env python

#-----------------------------------------------------------------------
# Swarm Tracker
# Duane Davis, 2014
#
# Defines an object that keeps track of all aircraft in a "swarm"
#-----------------------------------------------------------------------

#----------------------------
# Import a bunch of libraries
#----------------------------

# ROS imports
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

# Standard Python imports
import sys
from argparse import ArgumentParser

# Other ACS package imports
from autopilot_bridge.msg import LLA
from ap_network_bridge.msg import NetPoseStamped

# Control printing of messages to stdout
DBUG_PRINT = True
WARN_PRINT = True

# Base name for node topics and services
NODE_BASENAME = 'swarm_tracker'
ODOM_BASENAME = 'local_estim'
NET_BASENAME = 'network'


# Object for maintaining data about a single swarm member
# This object is primarily a container, so actual functionality (i.e., member
# functions) is minimal.  Manipulation of data members is through direct access
#
# Class member variables:
#   ID:  aircraft ID (integer) of the particular swarm member
#   lat: most recently reported latitude for this swarm element
#   lon: most recently reported longitude for this swarm element
#   alt: most recently reported altitude for this swarm element
#   roll: most recently reported roll (radians) for this swarm element
#   pitch: most recently reported pitch (radians) for this swarm element
#   yaw: most recently reported yaw (radians) for this swarm element
class SwarmElement(object):

    # Class initializer initializes member variables
    # @param latitude: initial latitude of the swarm element
    # @param longitude: initial longitude of the swarm element
    # @param altitude: initial altitude of the swarm element
    # @param roll: initial roll (radians) of the swarm element
    # @param pitch: initial pitch (radians) of the swarm element
    # @param yaw: initial yaw (radians) of the swarm element
    def __init__(self, ownID=1, latitude=0.0, longitude=0.0, altitude=0.0,\
                                roll=0.0, pitch=0.0, yaw=0.0):
        self.time = 0
        self.ID = ownID
        self.lat = latitude
        self.lon = longitude
        self.alt = altitude
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw



# Object that maintains the status of all known aircraft in the
# swarm.  Currently, this object just maintains the position of each member
# swarm member as it is broadcast over the network.
# TODO:  As functionality is added and additional information is available
# the functionality of this object can be increased accordingly
#
# Class member variables:
#   nodeName: name of the ROS node with which this object is associated
#   ownID:  ID (integer) of this particular aircraft
#   swarm: Dictionary of records for individual aircraft in the swarm
class SwarmTracker(object):


    # Class initializer initializes variables, subscribes to required
    # ROS topics, and creates required ros publishers.  Initializer
    # assumes that the object is already running within an initialized
    # ROS node (i.e., the object does not initialize itself as a node).
    # This enables multiple objects to run within a single node if
    # required.
    # @param ownID: ID (integer) of this particular aircraft
    # @param nodeName: name of the ROS node for this object
    # @param odomBaseName: base name of ROS topic to read odometry from
    # @param netBaseName: base name of ROS topic published by the network node
    def __init__(self, ownID, nodeName=NODE_BASENAME, \
                 odomBaseName=ODOM_BASENAME, netBaseName=NET_BASENAME):
        self.ownID = ownID
        self.nodeName = nodeName
        self.swarm = dict()
        self.swarm[self.ownID] = SwarmElement(self.ownID)

        # Set up object-specific ROS publishers
        # No publishers required in current implementation

        # Set up object-specific ROS subscribers
        rospy.Subscriber("%s/odom_combined"%odomBaseName, \
                         PoseWithCovarianceStamped, self.updateOwnPose)
        rospy.Subscriber("%s/recv_pose"%netBaseName, NetPoseStamped, \
                         self.updateSwarmPose)

        # Set up object-specific ROS services
        # No services required in current implementation


    #-----------------------------------------
    # ROS Subscriber callbacks for this object
    #-----------------------------------------

    # Updates swarm information for this aircraft when a new pose is
    # publishedHandles new pose information for this aircraft
    # @param poseMsg: PoseWithCovarianceStamped object with the new pose
    def updateOwnPose(self, poseMsg):
        try:
            element = self.swarm[self.ownID]
            element.time = float(poseMsg.header.stamp.secs) + \
                float(poseMsg.header.stamp.nsecs) / float(1e9)
            element.lat = poseMsg.pose.pose.position.x
            element.lon = poseMsg.pose.pose.position.y
            element.alt = poseMsg.pose.pose.position.z
            [ element.roll, element.pitch, element.yaw ] =\
                tf.transformations.euler_from_quaternion([ poseMsg.pose.pose.orientation.x,\
                                                           poseMsg.pose.pose.orientation.y,\
                                                           poseMsg.pose.pose.orientation.z,\
                                                           poseMsg.pose.pose.orientation.w ])
            self.log_dbug("update own pose: t=" + str(self.swarm[self.ownID].time) +\
                                    ", state=(" + str(self.swarm[self.ownID].lat) +\
                                           ", " + str(self.swarm[self.ownID].lon) +\
                                           ", " + str(self.swarm[self.ownID].alt) +\
                                           ", " + str(self.swarm[self.ownID].roll) +\
                                           ", " + str(self.swarm[self.ownID].pitch) +\
                                           ", " + str(self.swarm[self.ownID].yaw) + ")")
        except Exception as ex:
            self.log_warn("Callback error: " + ex.args[0])



    # Updates swarm information for a swarm aircraft when a new pose is
    # published
    # @param poseMsg: NetPoseStamped object with the new pose
    def updateSwarmPose(self, poseMsg):
        try:
            if not data.sender_id in self.swarm:
                self.swarm[poseMsg.sender_id] = SwarmElement(poseMsg.sender_id)

            # Ignore it if the we've already got a newer update
            poseTime = float(poseMsg.pose.header.stamp.secs) +\
                       float(poseMsg.pose.header.stamp.nsecs) / float(1e9)
            updateElement = swarm[poseMsg.sender_id]
            if poseTime <= updateElement.time: return

            # Update last-heard state
            updateElement.time = poseTime
            updateElement.lat = poseMsg.pose.pose.position.x
            updateElement.lon = poseMsg.pose.pose.position.y
            updateElement.alt = poseMsg.pose.pose.position.z
            [ element.roll, element.pitch, element.yaw ] =\
                tf.transformations.euler_from_quaternion([ poseMsg.pose.pose.orientation.x,\
                                                           poseMsg.pose.pose.orientation.y,\
                                                           poseMsg.pose.pose.orientation.z,\
                                                           poseMsg.pose.pose.orientation.w ])
            self.log_dbug("update aircraft " + str(poseMsg.sender_id) + \
                                   " state: t=" + str(updateElement.time) +\
                                     ", pose=(" + str(updateElement.lat) +\
                                           ", " + str(updateElement.lon) +\
                                           ", " + str(updateElement.alt) +\
                                           ", " + str(updateElement.roll) +\
                                           ", " + str(updateElement.pitch) +\
                                           ", " + str(updateElement.yaw) + ")")
        except Exception as ex:
            self.log_warn("Callback error: " + ex.args[0])



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



if __name__ == '__main__':
    # Parse command line arguments
    parser = ArgumentParser("rosrun ap_perception swarm_tracker.py")
    parser.add_argument('-n', "--nodename", dest="nodename", \
                        help="Name for the ROS node to register as", \
                        default=NODE_BASENAME)
    parser.add_argument('-id', "--aircraft", dest="acft", \
                        help="ID (integer) of this aircraft", default=1)
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])
    
    # Initialize ROS node & instantiate a SwarmTracker object
    rospy.init_node("swarm_tracker")
    tracker = SwarmTracker(int(args.acft))

    r = rospy.Rate(10.0) # Is 10hz a good rate?
    while not rospy.is_shutdown():
        # not doing anything other than looping for now
        r.sleep()


