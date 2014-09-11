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
import sys
from math import *
from argparse import ArgumentParser

# Other ACS package imports
from autopilot_bridge.msg import LLA
from ap_network_bridge.msg import NetPoseStamped
from acs import gps_utils as gps

# Control printing of messages to stdout
DBUG_PRINT = True
WARN_PRINT = True

# Base name for node topics and services
NODE_BASENAME = 'swarm_tracker'
ODOM_BASENAME = 'local_estim'
NET_BASENAME = 'network'

# Some constants to make functions more "reusable"
LAST_POSE = 0
CURRENT_POSE = 1
DR_POSE = 2


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
#   lastPose: aircraft pose at the previous discrete discrete sample
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
        self.lastPose = PoseWithCovarianceStamped()
        self.lastPose.header.seq = 0
        self.pose.header.stamp.secs = self.lastPose.header.stamp.secs = initT.secs
        self.pose.header.stamp.nsecs = self.lastPose.header.stamp.nsecs = initT.nsecs
        self.pose.pose.pose.position.x = self.lastPose.pose.pose.position.x = lat
        self.pose.pose.pose.position.y = self.lastPose.pose.pose.position.y = lon
        self.pose.pose.pose.position.z = self.lastPose.pose.pose.position.z = alt
        self.pose.pose.pose.orientation.x = self.lastPose.pose.pose.orientation.x = quatX
        self.pose.pose.pose.orientation.y = self.lastPose.pose.pose.orientation.y = quatY
        self.pose.pose.pose.orientation.z = self.lastPose.pose.pose.orientation.z = quatZ
        self.pose.pose.pose.orientation.w = self.lastPose.pose.pose.orientation.w = quatW
        self.pose.pose.covariance = self.lastPose.pose.covariance = ( 0.1, 0, 0, 0, 0, 0, \
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


    # Updates the object to contain a new pose.  The object's pose before
    # updating will be maintained as the previous pose (lastPose)
    # Displacement values are (x, y, z).  Orientation values are a 
    # quaternion (x, y, z, w).
    # TODO:  compute velocity based on previous and new pose
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
        # "Current pose" is now "last pose", new "current pose" from param values
        newLastPose = self.pose
        self.pose = self.lastPose
        self.lastPose = newLastPose
        self.pose.header.seq = self.lastPose.header.seq + 1
        self.pose.header.stamp.secs = newT.secs
        self.pose.header.stamp.nsecs = newT.nsecs
        self.pose.pose.pose.position.x = newLat
        self.pose.pose.pose.position.y = newLon
        self.pose.pose.pose.position.z = newAlt
        self.pose.pose.pose.orientation.x = quatX
        self.pose.pose.pose.orientation.y = quatY
        self.pose.pose.pose.orientation.z = quatZ
        self.pose.pose.pose.orientation.w = quatW

        # Differentiate last pose and new pose for velocities
        t1 = self.lastPose.header.stamp.secs * int(1e9) + self.lastPose.header.stamp.nsecs
        t2 = self.pose.header.stamp.secs * int(1e9) + self.pose.header.stamp.nsecs
        deltaT = (t2 - t1) / 1e9
        if deltaT <= 0: return
        pitch1, roll1, yaw1 = \
            tf.transformations.euler_from_quaternion(( self.lastPose.pose.pose.orientation.x,\
                                                       self.lastPose.pose.pose.orientation.y,\
                                                       self.lastPose.pose.pose.orientation.z,\
                                                       self.lastPose.pose.pose.orientation.w ))
        pitch2, roll2, yaw2 = self.getEulerAngles()
        deltaRoll, deltaPitch, deltaYaw = (pitch2 - pitch1), (roll2 - roll1), (yaw2 - yaw1)
        deltaZ = self.pose.pose.pose.position.x - self.lastPose.pose.pose.position.x
        self.velocity.twist.angular.x = deltaRoll / deltaT
        self.velocity.twist.angular.y = deltaPitch / deltaT
        self.velocity.twist.angular.z = deltaYaw / deltaT
        self.velocity.twist.linear.y = 0.0
        self.velocity.twist.linear.z = deltaZ / deltaT
        dist1to2 = gps.gps_distance(self.lastPose.pose.pose.position.x, \
                                    self.lastPose.pose.pose.position.y, \
                                    self.pose.pose.pose.position.x, \
                                    self.pose.pose.pose.position.y)
        dir1to2 = gps.gps_bearing(self.lastPose.pose.pose.position.x, \
                                  self.lastPose.pose.pose.position.y, \
                                  self.pose.pose.pose.position.x, \
                                  self.pose.pose.pose.position.y)
        if dist1to2 < 1.0: return  # too close to get a decent velocity estimate
#        if (abs(self.velocity.twist.angular.z) > 0):
#            deltaX = dist1to2 * sin(dir1to2)
#            deltaY = dist1to2 * cos(dir1to2)
#            self.velocity.twist.linear.x = \
#                deltaY / ((cos(yaw1) - cos(yaw2)) / self.velocity.twist.angular.z)
#        else:
        self.velocity.twist.linear.x = dist1to2 / deltaT



    # Computes a dead reckon position for a (presumably) future time
    # @param drTime time (rospy.rostime.Time object) for the DR computation
    def updateDRPose(self, drTime):
        t1 = self.pose.header.stamp.secs * int(1e9) + self.pose.header.stamp.nsecs
        t2 = drTime.secs * int(1e9) + drTime.nsecs
        deltaT = (t2 - t1) / 1e9
        if deltaT > 0.0:
            eulerX, eulerY, eulerZ = self.getEulerAngles(CURRENT_POSE)
            drEulerX = eulerX + deltaT * self.velocity.twist.angular.x
            drEulerY = eulerY + deltaT * self.velocity.twist.angular.y
            drEulerZ = eulerZ + deltaT * self.velocity.twist.angular.z
            self.drPose.header.stamp.secs = drTime.secs
            self.drPose.header.stamp.nsecs = drTime.nsecs
            # 2D DR of X and Y based on heading, forward velocity, turn rate, and deltaT
            xShift = (self.velocity.twist.linear.x / self.velocity.twist.angular.z) * \
                     (-sin(eulerZ) + sin(drEulerZ))
            yShift = (self.velocity.twist.linear.x / self.velocity.twist.angular.z) * \
                     (cos(eulerZ) - cos(drEulerZ))
            self.drPose.pose.pose.position.x, self.drPose.pose.pose.position.y = \
               gps.gps_offset(self.pose.pose.pose.position.x, \
                              self.pose.pose.pose.position.y, \
                              yShift, xShift)
            self.drPose.pose.pose.position.z = self.pose.pose.pose.position.z + \
                                               deltaT * self.velocity.twist.linear.z
            self.setPoseFromEulerAngles(drEulerX, drEulerY, drEulerZ, DR_POSE)
        else:   # Use current pose if requested DR time is in the past
            self.drPose.header.stamp.secs = self.pose.header.stamp.secs
            self.drPose.header.stamp.nsecs = self.pose.header.stamp.nsecs
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
        if whichPose == LAST_POSE: quaternion = self.lastPose.pose.pose.orientation
        elif whichPose == DR_POSE: quaternion = self.drPose.pose.pose.orientation
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
        if whichPose == LAST_POSE: pose = self.lastPose.pose.pose.position
        elif whichPose == DR_POSE: pose = self.drPose.pose.pose.position
        elif not whichPose == CURRENT_POSE: return None  # invalid request
        return ( pose.x, pose.y, pose.z )


    # Utility function to set pose orientation from Euler angles (roll, pitch yaw)
    # @param roll: Euler angle (radians) for rotation about the X axis
    # @param pitch: Euler angle (radians) for rotation about the Y axis
    # @param yaw: Euler angle (radians) for rotation about the Z axis
    # @param whichPose: enumeration (int) identifying previous, current, or DR pose
    def setPoseFromEulerAngles(self, roll, pitch, yaw, whichPose=CURRENT_POSE): 
        pose = self.pose.pose.pose.orientation
        if whichPose == LAST_POSE: pose = self.lastPose.pose.pose.orientation
        elif whichPose == DR_POSE: pose = self.drPose.pose.pose.orientation
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
            if data.sender_id in self.swarm:
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

