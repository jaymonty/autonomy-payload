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
from geometry_msgs.msg import TwistWithCovariance
from rospy import rostime

# Standard Python imports
from math import *
from argparse import ArgumentParser

# Other ACS package imports
from autopilot_bridge.msg import LLA
from ap_network_bridge.msg import NetPoseStamped
from ap_perception.msg import *
from ap_lib.gps_utils import *
from ap_lib.nodeable import *


# Control printing of messages to stdout
DBUG_PRINT = True
WARN_PRINT = True

# Base name for node topics and services
NODE_BASENAME = 'swarm_tracker'
SELF_ODOM_BASENAME = 'local_estim'
NET_ODOM_BASENAME = 'network'

# Some constants to make functions more "reusable"
CURRENT_POSE = 0
DR_POSE = 1


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
#   pose: aircraft pose in 6-DOF space (lat, lon, alt, phi, theta, psi)
#   drPose: computed (dead reckoning) position for a future time
#   velocity: aircraft velocity.  All angular rates and linear.x are in body
#             fixed coordinates (u, p, q, and r).  For now, linear.y is
#             ignored (assumed 0) and linear.z is earth fixed (climb rate)
#   TODO:  Fix velocity to work in 6-DOF body coordinates
class SwarmElement(object):

    # Class initializer initializes member variables (velocity is set to zero)
    # @param ownID: ID of the aircraft for this state vector
    # @param initT: time (rospy.rostime.Time object) of the initialized element
    # @param latitude: initial latitude of the swarm element
    # @param longitude: initial longitude of the swarm element
    # @param altitude: initial altitude of the swarm element
    # @param quatX: initial w value of the swarm element orientation quaternion
    # @param quatY: initial w value of the swarm element orientation quaternion
    # @param quatX: initial w value of the swarm element orientation quaternion
    # @param quatW: initial w value of the swarm element orientation quaternion
    def __init__(self, ownID=1, initT=rostime.Time(), lat=0.0, lon=0.0, alt=0.0, \
                       quatX=0.0, quatY=0.0, quatZ=0.0, quatW=1.0):
        self.ID = ownID

        # Initialize current and previous pose to same time and same position
        self.pose = PoseWithCovarianceStamped()
        self.pose.header.seq = 1
        self.pose.header.stamp.secs = initT.secs
        self.pose.header.stamp.nsecs = initT.nsecs
        self.pose.pose.pose.position.x = lat
        self.pose.pose.pose.position.y = lon
        self.pose.pose.pose.position.z = alt
        self.pose.pose.pose.orientation.x = quatX
        self.pose.pose.pose.orientation.y = quatY
        self.pose.pose.pose.orientation.z = quatZ
        self.pose.pose.pose.orientation.w = quatW
        self.pose.pose.covariance = ( 0.1, 0, 0, 0, 0, 0, \
                                      0, 0.1, 0, 0, 0, 0, \
                                      0, 0, 0.1, 0, 0, 0, \
                                      0, 0, 0, 0.1, 0, 0, \
                                      0, 0, 0, 0, 0.1, 0, \
                                      0, 0, 0, 0, 0, 0.1 )
        self.drPose = PoseWithCovarianceStamped()

        # Initialize the velocity to zero
        self.velocity = TwistWithCovariance()
        self.velocity.twist.linear.x = 0.0
        self.velocity.twist.linear.y = 0.0
        self.velocity.twist.linear.z = 0.0
        self.velocity.twist.angular.x = 0.0
        self.velocity.twist.angular.y = 0.0
        self.velocity.twist.angular.z = 0.0


    # Updates the object to contain a new pose.  Displacement
    # values are (x, y, z).  Orientation values are a 
    # quaternion (x, y, z, w).
    # TODO:  add velocity once it's being sent
    # @param newT: new time (rospy.rostime.Time object)
    # @param newX: new x value
    # @param newY: new y value
    # @param newZ: new z value
    # @param newOrientX: quaternion x value for the new pose
    # @param newOrientY: quaternion y value for the new pose
    # @param newOrientZ: quaternion z value for the new pose
    # @param newOrientW: quaternion w value for the new pose
    def updatePose(self, newT, newLat, newLon, newAlt, \
                         quatX, quatY, quatZ, quatW):
        self.pose.header.seq += 1
        self.pose.header.stamp.secs = newT.secs
        self.pose.header.stamp.nsecs = newT.nsecs
        self.pose.pose.pose.position.x = newLat
        self.pose.pose.pose.position.y = newLon
        self.pose.pose.pose.position.z = newAlt
        self.pose.pose.pose.orientation.x = quatX
        self.pose.pose.pose.orientation.y = quatY
        self.pose.pose.pose.orientation.z = quatZ
        self.pose.pose.pose.orientation.w = quatW

        # Once it's being sent, load velocities from message
        self.velocity.twist.linear.x = 0.0
        self.velocity.twist.linear.y = 0.0
        self.velocity.twist.linear.z = 0.0
        self.velocity.twist.angular.x = 0.0
        self.velocity.twist.angular.y = 0.0
        self.velocity.twist.angular.z = 0.0



    # Computes a dead reckon position for a (presumably) future time
    # @param drTime time (rospy.rostime.Time object) for the DR computation
    # TODO:  Not yet implemented
    def updateDRPose(self, drTime):
        poseTimeInt = float(self.pose.header.stamp.secs) +\
                      float(self.pose.header.stamp.nsecs) / float(1e9)
        drTimeInt = float(drTime.secs) + float(drTime.nsecs) / float(1e9)
        if drTimeInt <= poseTimeInt:  # If DR time in past, use current pose
            self.drPose.header.stamp.secs = self.pose.header.stamp.secs
            self.drPose.header.stamp.nsecs = self.pose.header.stamp.nsecs
            self.drPose.pose.pose.position.x = self.pose.pose.pose.position.x
            self.drPose.pose.pose.position.y = self.pose.pose.pose.position.y
            self.drPose.pose.pose.position.z = self.pose.pose.pose.position.z
            self.drPose.pose.pose.orientation.x = self.pose.pose.pose.orientation.x
            self.drPose.pose.pose.orientation.y = self.pose.pose.pose.orientation.y
            self.drPose.pose.pose.orientation.z = self.pose.pose.pose.orientation.z
            self.drPose.pose.pose.orientation.w = self.pose.pose.pose.orientation.w
        else:
            self.drPose.header.stamp.secs = drTime.secs
            self.drPose.header.stamp.nsecs = drTime.nsecs
            self.drPose.pose.pose.position.x = self.pose.pose.pose.position.x
            self.drPose.pose.pose.position.y = self.pose.pose.pose.position.y
            self.drPose.pose.pose.position.z = self.pose.pose.pose.position.z
            self.drPose.pose.pose.orientation.x = self.pose.pose.pose.orientation.x
            self.drPose.pose.pose.orientation.y = self.pose.pose.pose.orientation.y
            self.drPose.pose.pose.orientation.z = self.pose.pose.pose.orientation.z
            self.drPose.pose.pose.orientation.w = self.pose.pose.pose.orientation.w


    # Utility function to extract Euler angles (roll, pitch, yaw) of a pose
    # @param whichPose:  enumeration (int) identifying previous, current, or DR pose
    # @return 3-vector containing requested pose Euler angles
    def getEulerAngles(self, whichPose=CURRENT_POSE):
        quaternion = self.pose.pose.pose.orientation
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
        pose = self.pose.pose.pose.position
        if whichPose == DR_POSE: pose = self.drPose.pose.pose.position
        elif not whichPose == CURRENT_POSE: return None  # invalid request
        return ( pose.x, pose.y, pose.z )


    # Utility function to set pose orientation from Euler angles (roll, pitch yaw)
    # @param roll: Euler angle (radians) for rotation about the X axis
    # @param pitch: Euler angle (radians) for rotation about the Y axis
    # @param yaw: Euler angle (radians) for rotation about the Z axis
    # @param whichPose: enumeration (int) identifying previous, current, or DR pose
    def setPoseFromEulerAngles(self, roll, pitch, yaw, whichPose=CURRENT_POSE): 
        pose = self.pose.pose.pose.orientation
        if whichPose == DR_POSE: pose = self.drPose.pose.pose.orientation
        elif not whichPose == CURRENT_POSE: return  # invalid request
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.x = quat[0]
        pose.y = quat[1]
        pose.z = quat[2]
        pose.w = quat[3]



# Object that maintains the status of all known aircraft in the
# swarm.  Currently, this object just maintains the position and velocity of
# each swarm member as it is broadcast over the network.  Data for each element
# is maintained in a dictionary object with 1 record for each swarm member.
# TODO:  As functionality is added and additional information is available
# the functionality of this object can be increased accordingly
#
# Class member variables:
#   nodeName: name of the ROS node with which this object is associated
#   ownID:  ID (integer) of this particular aircraft
#   swarm: Dictionary of records for individual aircraft in the swarm
#   swarmPublisher: Object for publishing swarm state to the ROS topic
#   swarmMessage: Container for swarm states to be published to the ROS topic
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
        self.swarm[self.ownID] = SwarmElement(self.ownID)
        self.swarmPublisher = None
        self.swarmMessage = SwarmStateStamped()
        self.swarmMessage.header.seq = 0



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
            vehicle.updateDRPose(self.swarmMessage.header.stamp)
            vehicleMsg = SwarmVehicleState()
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
    def updateOwnPose(self, poseMsg):
        try:
            element = self.swarm[self.ownID]
            newTime = poseMsg.header.stamp
            element.updatePose(newTime, poseMsg.pose.pose.position.x, \
                                        poseMsg.pose.pose.position.y, \
                                        poseMsg.pose.pose.position.z, \
                                        poseMsg.pose.pose.orientation.x, \
                                        poseMsg.pose.pose.orientation.y, \
                                        poseMsg.pose.pose.orientation.z, \
                                        poseMsg.pose.pose.orientation.w)

            (roll, pitch, yaw) = element.getEulerAngles()
            self.log_dbug("update own pose: t=" + str(element.pose.header.stamp) + "\n" + \
                      "pose=(" + str(element.pose.pose.pose.position.x) + \
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
            self.log_warn("Self update callback error: " + ex.args[0])



    # Updates swarm information for a swarm aircraft when a new pose is published
    # @param poseMsg: NetPoseStamped object with the new pose
    def updateSwarmPose(self, poseMsg):
        try:
            poseTime = float(poseMsg.pose.header.stamp.secs) +\
                       float(poseMsg.pose.header.stamp.nsecs) / float(1e9)

            # Update an existing element if it's already in the dictionary
            if poseMsg.sender_id in self.swarm:
                updateElement = swarm[poseMsg.sender_id]
                if poseTime <= updateElement.time: return # older than latest data
                updateElement.updatePose(newTime, poseMsg.pose.pose.position.x, \
                                                  poseMsg.pose.pose.position.y, \
                                                  poseMsg.pose.pose.position.z, \
                                                  poseMsg.pose.pose.orientation.x, \
                                                  poseMsg.pose.pose.orientation.y, \
                                                  poseMsg.pose.pose.orientation.z, \
                                                  poseMsg.pose.pose.orientation.w)

            else:  # Create and initialize a new element if this is the first report
                newElement = SwarmElement(poseMsg.sender_id, poseTime, poseMsg.pose.pose.position.x, \
                                                                       poseMsg.pose.pose.position.y, \
                                                                       poseMsg.pose.pose.position.z, \
                                                                       poseMsg.pose.pose.orientation.x, \
                                                                       poseMsg.pose.pose.orientation.y, \
                                                                       poseMsg.pose.pose.orientation.z, \
                                                                       poseMsg.pose.pose.orientation.w)
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



