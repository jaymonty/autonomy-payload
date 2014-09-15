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
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from rospy import rostime

# Standard Python imports
from math import *
from argparse import ArgumentParser

# Other ACS package imports
from autopilot_bridge.msg import LLA
from ap_network_bridge.msg import NetPoseStamped
from ap_msgs.msg import *
from ap_lib.gps_utils import *
from ap_lib.nodeable import *


# Base name for node topics and services
NODE_BASENAME = 'swarm_tracker'
SELF_ODOM_BASENAME = 'local_estim'
NET_ODOM_BASENAME = 'network'

# Some constants to make functions more "reusable"
CURRENT_POSE = 0
DR_POSE = 1


# Convenience object for nodes that need to subscribe to the ROS topics that
# the SwarmTracker is publishing to.  The object sets up the subscriber,
# maintains the state of the swarm and provides accessors for the swarm data
#
# Class member variables:
#   timestamp: timestamp object for the time the swarm status was updated
#   swarm: dictionary object containing individual swarm aircraft data (SwarmElement)
class SwarmTrackerSubscriber(object):

    # Initializer sets empty objects for maintained variables and registers
    # as a subscriber to the topics published by the SwarmTracker class
    # @param baseName: ROS basename for the topic being subscribed to
    def __init__(self, baseName=NODE_BASENAME):
        self.timestamp = rospy.Time.now()
        self.swarm = dict()
        rospy.Subscriber("%s/swarm_state"%baseName, \
                         SwarmStateStamped, self.updateSwarmState)


    # Callback to update swarm information when it is received
    # @param swarmMsg: message containing received swarm data
    def updateSwarmState(self, swarmMsg):
        lastSwarm = self.swarm
        self.swarm = dict()
        self.timestamp = swarmMsg.header.stamp
        for swarmElement in swarmMsg.swarm:
            if swarmElement.vehicleID in lastSwarm:
                element = lastSwarm[swarmElement.vehicleID]
                element.updatePose(swarmElement.pose, self.timestamp)
                element.updateVelocity(swarmElement.velocity)
                self.swarm[swarmElement.vehicleID] = element
            else:
                element = SwarmElement(swarmElement.vehicleID, \
                                       swarmElement.pose, self.timestamp)
                element.updateVelocity(swarmElement.velocity)
                self.swarm[swarmElement.vehicleID] = element
        del lastSwarm


    # Returns a record (SwarmElement object) for a requested swarm member
    # or None if the requested swarm element does not exist
    # @param vehicleID: vehicle ID (int) of the requested vehicle
    def getRecord(self, vehicleID):
        if vehicleID in self.swarm: return self.swarm[vehicleID]
        else: return None


###############################################################################

# Object for maintaining data about a single swarm member.  Uses a ROS
# PoseWithCovarianceStamped object to represent vehicle pose (and previous
# pose) and a ROS TwistWithCovariance to represent velocity.
# Provides some functionality within the class, but access to individual data
# members is still largely through direct access as opposed to member functions
# Covariance values are simple defaults for now and aren't maintained with any
# rigor.  Eventually, noise will need to be characterized so that the object
# can be used as part of a Kalman Filter (or EKF).
#
# Class member variables:
#   ID:  aircraft ID (integer) of the particular swarm member
#   stamp: header information for the vehicle state (Header)
#   pose: aircraft pose in 7-DOF space (PoseWithCovariance)
#   drPose: computed (dead reckoning) position for a future time (PoseWithCovarianceStamped)
#   velocity: aircraft velocity.  Angular rates are in the body-fixed
#             frame (p, q, and r).  Linear rates are in the earth-fixed
#             north-east-down frame (xDot, yDot, zDot) (TwistWithCovariance)
#
# Class member functions
#   updatePose:  update the pose information with new data
#   updateVelocity:  update the velocity information with new data
#   computeDRPose: computes a predicted pose based on velocity & time elapsed
#   getEulerAngles: returns a 3-vector of Euler angles
#   getPosition: returns a 3-vector (lat, lon, alt) position
#   setPoseFromEulerAngles: sets the quaternion orientation
#
#   TODO:  Implement dead reckoning
class SwarmElement(object):

    # Class initializer initializes member variables (velocity is set to zero)
    # @param ownID: ID of the aircraft for this state vector
    # @param initPose: PoseWithCovariance object with the initial pose
    # @param initT: time (rospy.rostime.Time object) of the initialized element
    def __init__(self, ownID, initPose, initT=rostime.Time()):
        self.ID = ownID
        self.stamp = Header()
        self.stamp.seq = 0
        self.stamp.stamp = initT
        self.pose = initPose
        self.velocity = TwistWithCovariance() # Initial velocity = zero
        self.drPose = PoseWithCovarianceStamped()


    # Updates the object to contain a new pose.  Displacement
    # values are (x, y, z).  Orientation values are a 
    # quaternion (x, y, z, w).
    # @param newPose: PoseWithCovariance object with the new pose
    # @param time: timestamp of the new pose
    def updatePose(self, newPose, time):
        oldPose = self.pose
        self.stamp.seq += 1
        self.stamp.stamp = time
        self.pose = newPose
        del oldPose


    # Updates the velocity portion of the vehicle state
    # @param newVelocity: twist with covariance message with the new velocity
    def updateVelocity(self, newVelocity):
        oldVelocity = self.velocity
        self.velocity = newVelocity
        del oldVelocity


    # Computes a dead reckon position for a (presumably) future time
    # @param drTime time (rospy.rostime.Time object) for the DR computation
    # TODO:  Not yet implemented
    def computeDRPose(self, drTime):
        poseTimeInt = float(self.stamp.stamp.secs) +\
                      float(self.stamp.stamp.nsecs) / float(1e9)
        drTimeInt = float(drTime.secs) + float(drTime.nsecs) / float(1e9)
        if drTimeInt <= poseTimeInt:  # If DR time in past, use current pose
            self.drPose.header.stamp.secs = self.header.stamp.secs
            self.drPose.header.stamp.nsecs = self.header.stamp.nsecs
            self.drPose.pose.pose.position.x = self.pose.pose.position.x
            self.drPose.pose.pose.position.y = self.pose.pose.position.y
            self.drPose.pose.pose.position.z = self.pose.pose.position.z
            self.drPose.pose.pose.orientation.x = self.pose.pose.orientation.x
            self.drPose.pose.pose.orientation.y = self.pose.pose.orientation.y
            self.drPose.pose.pose.orientation.z = self.pose.pose.orientation.z
            self.drPose.pose.pose.orientation.w = self.pose.pose.orientation.w
        else:
            self.drPose.header.stamp.secs = drTime.secs
            self.drPose.header.stamp.nsecs = drTime.nsecs
            self.drPose.pose.pose.position.x = self.pose.pose.position.x
            self.drPose.pose.pose.position.y = self.pose.pose.position.y
            self.drPose.pose.pose.position.z = self.pose.pose.position.z
            self.drPose.pose.pose.orientation.x = self.pose.pose.orientation.x
            self.drPose.pose.pose.orientation.y = self.pose.pose.orientation.y
            self.drPose.pose.pose.orientation.z = self.pose.pose.orientation.z
            self.drPose.pose.pose.orientation.w = self.pose.pose.orientation.w


    # Utility function to extract Euler angles (roll, pitch, yaw) of a pose
    # @param whichPose:  enumeration (int) identifying previous, current, or DR pose
    # @return 3-vector containing requested pose Euler angles
    def getEulerAngles(self, whichPose=CURRENT_POSE):
        quaternion = self.pose.pose.orientation
        if whichPose == DR_POSE: quaternion = self.drPose.pose.pose.orientation
        elif not whichPose == CURRENT_POSE: return None  # invalid request
        return tf.transformations.euler_from_quaternion(( quaternion.x,\
                                                          quaternion.y,\
                                                          quaternion.z,\
                                                          quaternion.w ))


    # Utility function to extract position (lat, lon, alt) of a pose
    # @param whichPose: enumeration (int) identifying previous, current, or DR pose
    # @return 3-vector containing the requested position information
    def getPosition(self, whichPose=CURRENT_POSE):
        pose = self.pose.pose.position
        if whichPose == DR_POSE: pose = self.drPose.pose.pose.position
        elif not whichPose == CURRENT_POSE: return None  # invalid request
        return ( pose.x, pose.y, pose.z )


    # Utility function to set pose orientation from Euler angles (roll, pitch yaw)
    # @param roll: Euler angle (radians) for rotation about the X axis
    # @param pitch: Euler angle (radians) for rotation about the Y axis
    # @param yaw: Euler angle (radians) for rotation about the Z axis
    # @param whichPose: enumeration (int) identifying previous, current, or DR pose
    def setPoseFromEulerAngles(self, roll, pitch, yaw, whichPose=CURRENT_POSE): 
        pose = self.pose.pose.orientation
        if whichPose == DR_POSE: pose = self.drPose.pose.pose.orientation
        elif not whichPose == CURRENT_POSE: return  # invalid request
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.x = quat[0]
        pose.y = quat[1]
        pose.z = quat[2]
        pose.w = quat[3]


###############################################################################

# Object that maintains the status of all known aircraft in the
# swarm.  Currently, this object just maintains the position and velocity of
# each swarm member as it is broadcast over the network.  Data for each element
# is maintained in a dictionary object with 1 record for each swarm member.
#
# Class member variables:
#   nodeName: name of the ROS node with which this object is associated
#   ownID:  ID (integer) of this particular aircraft
#   swarm: Dictionary of records for individual aircraft in the swarm
#   swarmPublisher: Object for publishing swarm state to the ROS topic
#   swarmMessage: Container for swarm states to be published to the ROS topic
#
# Class methods
#   callbackSetup: Nodeable class virtual implementation to set up subscriptions
#   publisherSetup: Nodeable class function implementation to set up publishers
#   executeTimedLoop: Nodeable class function implementation for one loop iteration
#   updateOwnPose: callback when an updated pose for this aircraft is published
#   updateSwarmPose: callback when an updated pose for a swarm aircraft is published
#   drSwarm: computes a DR position for every aircraft in the swarm
#
# TODO:  As functionality is added and additional information is available
# the functionality of this object will be increased accordingly
class SwarmTracker(Nodeable):

    # Class initializer initializes variables, subscribes to required
    # ROS topics, and creates required ros publishers.  Initializer
    # assumes that the object is already running within an initialized
    # ROS node (i.e., the object does not initialize itself as a node).
    # This enables multiple objects to run within a single node if rqd.
    # @param ownID: ID (integer) of this particular aircraft
    # @param nodeName: name of the ROS node for this object
    # @param odomBaseName: base name of ROS topic to read odometry from
    # @param netBaseName: base name of ROS topic published by the network node
    def __init__(self, ownID, nodeName=NODE_BASENAME):
        Nodeable.__init__(self, nodeName)
        self.ownID = ownID
        self.swarm = dict()
        self.swarm[self.ownID] = SwarmElement(self.ownID, PoseWithCovariance())
        self.swarmPublisher = None
        self.swarmMessage = SwarmStateStamped()
        self.swarmMessage.header.seq = 0
#        self.DBUG_PRINT = True
#        self.WARN_PRINT = True



    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Establishes the callbacks for the SwarmTracker object.  The object
    # subscribes to the odom_combined topic for own-aircraft state updates
    # and the recv_pose topic for swarm member updates
    # @param params: list as follows: [ odometry_base_name, net_base_name ]
    def callbackSetup(self, params=[ SELF_ODOM_BASENAME, NET_ODOM_BASENAME ]):
        rospy.Subscriber("%s/odom_combined"%params[0], \
                         PoseWithCovarianceStamped, self.updateOwnPose)
        rospy.Subscriber("%s/recv_pose"%params[1], NetPoseStamped, \
                         self.updateSwarmPose)


    # Sets up publishers for the SwarmTracker object.  The object publishes
    # SwarmStatus messages to the swarm_state topic
    # @param params: list of parameters (none required for this method)
    def publisherSetup(self, params=[]):
        self.swarmPublisher = \
            rospy.Publisher("%s/swarm_state"%self.nodeName, SwarmStateStamped)


    # Executes one iteration of the timed loop for the SwarmTracker object
    # The loop computes a DR position for each vehicle in the swarm for the
    # current time and publishes a SwarmState message to the ROS topic.
    def executeTimedLoop(self):
        # get current time--TODO:  how to get better time?
        self.swarmMessage.header.stamp = rospy.Time.now()
        del self.swarmMessage.swarm[:]
        for vID in self.swarm:
            vehicle = self.swarm[vID]
            vehicle.computeDRPose(self.swarmMessage.header.stamp)
            vehicleMsg = SwarmVehicleState()
            vehicleMsg.vehicleID = vID
            vehicleMsg.pose = vehicle.drPose.pose
            vehicleMsg.velocity = vehicle.velocity
            self.swarmMessage.swarm.append(vehicleMsg)
        self.swarmPublisher.publish(self.swarmMessage)
        self.swarmMessage.header.seq += 1


    #-----------------------------------------
    # ROS Subscriber callbacks for this object
    #-----------------------------------------

    # Updates swarm information for this aircraft when a new pose is
    # publishedHandles new pose information for this aircraft
    # @param poseMsg: PoseWithCovarianceStamped object with the new pose
    # TODO:  change to include velocity when it's included in the msg
    def updateOwnPose(self, poseMsg):
        try:
            element = self.swarm[self.ownID]
            newTime = poseMsg.header.stamp
            element.updatePose(poseMsg.pose, newTime)
            element.updateVelocity(TwistWithCovariance())
            (roll, pitch, yaw) = element.getEulerAngles()
            self.log_dbug("update own pose: t=" + str(element.stamp.stamp) + "\n" + \
                      "pose=(" + str(element.pose.pose.position.x) + \
                          ", " + str(element.pose.pose.position.y) + \
                          ", " + str(element.pose.pose.position.z) + \
                          ", " + str(roll) + ", " + str(pitch) + ", " + str(yaw) + ")\n" + \
                       "vel=(" + str(element.velocity.twist.linear.x) + \
                          ", " + str(element.velocity.twist.linear.y) + \
                          ", " + str(element.velocity.twist.linear.z) + \
                          ", " + str(element.velocity.twist.angular.x) + \
                          ", " + str(element.velocity.twist.angular.y) + \
                          ", " + str(element.velocity.twist.angular.z) + ")")
        except Exception as ex:
            self.log_warn("Self update callback error: " + ex.args[0])



    # Updates swarm information for a swarm aircraft when a new pose is published
    # @param poseMsg: NetPoseStamped object with the new pose
    # TODO: change to include velocity when it's included in the msg
    def updateSwarmPose(self, poseMsg):
        try:
            newTime = poseMsg.pose.header.stamp
            poseTime = std_msgs.secs + (std_msgs.nsecs / float(1e9))

            # Update an existing element if it's already in the dictionary
            if poseMsg.sender_id in self.swarm:
                updateElement = swarm[poseMsg.sender_id]
                elTime = float(updateElement.header.stamp.secs) +\
                         float(updateElement.header.stamp.nsecs) / float(1e9)
                if poseTime <= elTime: return # older than latest data
                updateElement.updatePose(poseMsg.pose.pose, newTime)
                element.updateVelocity(TwistWithCovariance())
            else:  # Create and initialize a new element if this is the first report
                newElement = SwarmElement(poseMsg.sender_id, poseMsg.pose, poseTime)
                newELement.updateVelocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                self.swarm[poseMsg.sender_id] = newElement

            element = swarm[poseMsg.sender_id]
            (roll, pitch, yaw) = element.pose.getEulerAngles()
            self.log_dbug("update aircraft " + str(poseMsg.sender_id) + ", t=" + \
                          str(element.pose.header.stamp) + "\n" + \
                    ", pose=(" + str(element.pose.pose.pose.position.x) + \
                          ", " + str(element.pose.pose.pose.position.y) + \
                          ", " + str(element.pose.pose.pose.position.z) + \
                          ", " + str(roll) + ", " + str(pitch) + ", " + str(yaw) + ")\n" + \
                       "vel=(" + str(element.velocity.twist.linear.x) + \
                          ", " + str(element.velocity.twist.linear.y) + \
                          ", " + str(element.velocity.twist.linear.z) + \
                          ", " + str(element.velocity.twist.angular.x) + \
                          ", " + str(element.velocity.twist.angular.y) + \
                          ", " + str(element.velocity.twist.angular.z) + ")")

        except Exception as ex:
            self.log_warn("Swarm update callback error: " + ex.args[0])


    #--------------------------------------
    # Class-specific functions
    #--------------------------------------

    # Computes a DR position for each element in the swarm for a specific time
    # @param drTime time (rospy.rostime.Time object) for the DR computation
    def drSwarm(self, drTime):
        for acft in self.swarm:
            self.swarm[acft].updateDRPose(drTime)

