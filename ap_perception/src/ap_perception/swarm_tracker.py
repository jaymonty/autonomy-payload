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
import autopilot_bridge.msg as apbrg
from rospy import rostime

# Standard Python imports
from math import *
from threading import RLock

# Other ACS package imports
#from autopilot_bridge.msg import LLA
from ap_msgs.msg import *
from ap_lib.gps_utils import *
from ap_lib.quaternion_math import *
from ap_lib.nodeable import *


# Base name for node topics and services
NODE_BASENAME = 'swarm_tracker'

# Some constants to make functions more "reusable"
CURRENT_POSE = 0
DR_POSE = 1

MAX_DR_TIME = rospy.Duration(5.0)  # Max time for a DR position update


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
        rospy.Subscriber("%s/swarm_uav_states"%baseName, \
                         SwarmStateStamped, self.updateSwarmState)


    # Callback to update swarm information when it is received
    # @param swarmMsg: message containing received swarm data
    def updateSwarmState(self, swarmMsg):
        self.timestamp = swarmMsg.header.stamp
        for msgElement in swarmMsg.swarm:
            if msgElement.vehicle_id in self.swarm:
                element = self.swarm[msgElement.vehicle_id]
                element.updateState(msgElement.state, msgElement.subswarm_id)
            else:
                element = SwarmElement(msgElement.vehicle_id, \
                                       msgElement.subswarm_id, \
                                       msgElement.state)
                self.swarm[msgElement.vehicle_id] = element


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
#   subSwarmID:  Subswarm to which this SwarmElement is assigned
#   swarmState:  The current swarm state for this vehicle
#   swarmBehavior:  The currently active swarm behavior for this vehicle
#   isActive:  Indicates that the vehicle is active (recent update)
#   state: aircraft pose and  (Geodometry)
#   drPose: computed (dead reckoning) position for a future time (Geodometry)
#   _stateMsg: container object for the swarm state message (SwarmVehicleState)
#
# Class member functions
#   updateState:  update the state information with new data
#   updateVelocity:  update the velocity information with new data
#   computeDRPose: computes a predicted pose based on velocity & time elapsed
#   getEulerAngles: returns a 3-vector of Euler angles
#   getPosition: returns a 3-vector (lat, lon, alt) position
#   setPoseFromEulerAngles: sets the quaternion orientation
#
#   TODO:  Implement nonlinear dead reckoning
class SwarmElement(object):

    # Class initializer initializes member variables (velocity is set to zero)
    # @param ownID: ID of the aircraft for this state vector
    # @param subswarm: ID of the subswarm to which this aircraft belongs
    # @param initState: Geodometry object with the initial state information
    # @param swarmState
    # @param baseAlt: MSL altitude from which relative altitude (rel_alt) is calculated
    def __init__(self, ownID, subswarm, initState, baseAlt = 0.0):
        self.ID = ownID
        self.subSwarmID = subswarm
        self.swarmState = 0
        self.swarmBehavior = 0
        self.isActive = False
        self.state = initState
        self.state.header.seq = 0
        self.state.header.frame_id = 'base_footprint'
        if not initState.pose.pose.position.using_rel_alt:
            self.state.pose.pose.position.rel_alt = \
                initState.pose.pose.position.alt - baseAlt
        self.state.pose.pose.position.using_alt = True
        self.state.pose.pose.position.using_rel_alt = True
        self.drPose = apbrg.Geodometry()
        self.drPose.header.seq = 0
        self.state.header.frame_id = 'base_footprint'
        self.drPose.pose.pose.orientation.w = 1.0
        self.drPose.pose.pose.position.using_alt = True
        self.drPose.pose.pose.position.using_rel_alt = True
        self._stateMsg = SwarmVehicleState()
        self._stateMsg.vehicle_id = self.ID
        self._stateMsg.subswarm_id = self.subSwarmID
        self._stateMsg.state.pose.pose.position.using_alt = True
        self._stateMsg.state.pose.pose.position.using_rel_alt = True


    # Updates the object to contain new state information. Displacement
    # values are (x, y, z).  Orientation values are a 
    # quaternion (x, y, z, w).  Linear velocity data is earth-fixed
    # meters per second (vx, vy, vz).  Angular velocity is body-fixed
    # radians per second (p, q, r)
    # @param newState: Geodometry object with the new state information
    # @param subswarm: ID of the subswarm to which this swarm member belongs
    # @param swarmState: Updated vehicle's current swarm state
    # @param swarmBehavior: Updated vehicle's currently active swarm behavior
    def updateState(self, newState, subswarm, swarmState, swarmBehavior):
        self.isActive = True
        self.subSwarmID = subswarm
        self.swarmState = swarmState
        self.swarmBehavior = swarmBehavior
        self.state = newState
        self.state.header.frame_id = 'base_footprint'
        self._stateMsg.subswarm_id = subswarm
        self._stateMsg.swarm_state = swarmState
        self._stateMsg.swarm_behavior = swarmBehavior


    # Computes a dead reckon position for a (presumably) future time
    # @param drTime time (rospy.rostime.Time object) for the DR computation
    # @param baseAlt altitude from which rel_alt is calculated (if not built in)
    # TODO:  DR is a simple linear update--add in angular rates and make it an arc
    # TODO:  Implement covariance growth functionality
    def computeDRPose(self, drTime):
        poseTimeFloat = float(self.state.header.stamp.secs) +\
                        float(self.state.header.stamp.nsecs) / float(1e9)
        drTimeFloat = float(drTime.secs) + float(drTime.nsecs) / float(1e9)
        dt = drTimeFloat - poseTimeFloat
        if dt <= 0.0:  # If DR time in past, use current pose
            self.drPose.header.stamp.secs = self.state.header.stamp.secs
            self.drPose.header.stamp.nsecs = self.state.header.stamp.nsecs
            self.drPose.pose.pose.position.lat = self.state.pose.pose.position.lat
            self.drPose.pose.pose.position.lon = self.state.pose.pose.position.lon
            self.drPose.pose.pose.position.alt = self.state.pose.pose.position.alt
            self.drPose.pose.pose.position.rel_alt = self.state.pose.pose.position.rel_alt
            self.drPose.pose.pose.orientation.x = self.state.pose.pose.orientation.x
            self.drPose.pose.pose.orientation.y = self.state.pose.pose.orientation.y
            self.drPose.pose.pose.orientation.z = self.state.pose.pose.orientation.z
            self.drPose.pose.pose.orientation.w = self.state.pose.pose.orientation.w
            self.drPose.pose.covariance = self.state.pose.covariance
        else:
            self.drPose.header.stamp.secs = drTime.secs
            self.drPose.header.stamp.nsecs = drTime.nsecs

            # Dead Reckon the orientation quaternion
            # TODO:  check this once the angular velocity numbers are verified
            poseQuaternion = [ self.state.pose.pose.orientation.x, \
                               self.state.pose.pose.orientation.y, \
                               self.state.pose.pose.orientation.z, \
                               self.state.pose.pose.orientation.w ]
            pqrQuaternion = [ self.state.twist.twist.angular.x, \
                              self.state.twist.twist.angular.y, \
                              self.state.twist.twist.angular.z, 0.0 ]
            qDot = multiply_quaternion( poseQuaternion, pqrQuaternion)
            qDot = scalar_multiply_quaternion(qDot, 0.5)
            poseQuaternion = \
                [ qDot[0] * dt + self.state.pose.pose.orientation.x, \
                  qDot[1] * dt + self.state.pose.pose.orientation.y, \
                  qDot[2] * dt + self.state.pose.pose.orientation.z, \
                  qDot[3] * dt + self.state.pose.pose.orientation.w ]
            poseQuaternion = \
                normalize_quaternion(poseQuaternion)
            self.drPose.pose.pose.orientation.x = poseQuaternion[0]
            self.drPose.pose.pose.orientation.y = poseQuaternion[1]
            self.drPose.pose.pose.orientation.z = poseQuaternion[2]
            self.drPose.pose.pose.orientation.w = poseQuaternion[3]

            # simple straight line DR for now (use a curve at some point)
            direction = atan2(self.state.twist.twist.linear.y, \
                              self.state.twist.twist.linear.x)
            velocity = math.hypot(self.state.twist.twist.linear.y, \
                                  self.state.twist.twist.linear.x)
            distance = velocity * dt
            (self.drPose.pose.pose.position.lat, self.drPose.pose.pose.position.lon) = \
                gps_newpos(self.state.pose.pose.position.lat, \
                           self.state.pose.pose.position.lon, \
                           direction, distance)
#            base_alt = self.state.pose.pose.position.alt - \
#                       self.state.pose.pose.position.rel_alt
#            self.drPose.pose.pose.position.alt = self.state.pose.pose.position.alt - \
#                                                 (self.state.twist.twist.linear.z * dt)
#            self.drPose.pose.pose.position.rel_alt = \
#                self.drPose.pose.pose.position.alt - base_alt
            # Not DRing altitude to avoid "burble" effect causing larger errors
            self.drPose.pose.pose.position.alt = self.state.pose.pose.position.alt
            self.drPose.pose.pose.position.rel_alt = self.state.pose.pose.position.rel_alt
            self.drPose.pose.covariance = self.state.pose.covariance


    # Utility function to extract Euler angles (roll, pitch, yaw) of a pose
    # @param whichPose:  enumeration (int) identifying previous, current, or DR pose
    # @return 3-vector containing requested pose Euler angles
    def getEulerAngles(self, whichPose=CURRENT_POSE):
        quaternion = self.state.pose.pose.orientation
        if whichPose == DR_POSE: quaternion = self.drPose.pose.pose.orientation
        elif not whichPose == CURRENT_POSE: return None  # invalid request
        return tf.transformations.euler_from_quaternion(( quaternion.x,\
                                                          quaternion.y,\
                                                          quaternion.z,\
                                                          quaternion.w ))


    # Utility function to extract position (lat, lon, alt, rel_alt) of a pose
    # @param whichPose: enumeration (int) identifying previous, current, or DR pose
    # @return 3-vector containing the requested position information
    def getPosition(self, whichPose=CURRENT_POSE):
        pose = self.state.pose.pose.position
        if whichPose == DR_POSE: pose = self.drPose.pose.pose.position
        elif not whichPose == CURRENT_POSE: return None  # invalid request
        return ( pose.lat, pose.lon, pose.alt, pose.rel_alt )


    # Utility function to set pose orientation from Euler angles (roll, pitch yaw)
    # @param roll: Euler angle (radians) for rotation about the X axis
    # @param pitch: Euler angle (radians) for rotation about the Y axis
    # @param yaw: Euler angle (radians) for rotation about the Z axis
    # @param whichPose: enumeration (int) identifying previous, current, or DR pose
    def setPoseFromEulerAngles(self, roll, pitch, yaw, whichPose=CURRENT_POSE): 
        pose = self.state.pose.pose.orientation
        if whichPose == DR_POSE: pose = self.drPose.pose.pose.orientation
        elif not whichPose == CURRENT_POSE: return  # invalid request
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.x = quat[0]
        pose.y = quat[1]
        pose.z = quat[2]
        pose.w = quat[3]


    # Utility function to generate a string representation of the state as
    # "t=####, (Lat, Lon, Alt, Rel_Alt, Qx, Qy, Qz, Qw, Vx, Vy, Vz, p, q, r)"
    # @return the requested string form of the state vector
    def getAsString(self):
        return "t=" + str(self.state.header.stamp) + ", (" + \
                      str(self.state.pose.pose.position.lat) + ", " + \
                      str(self.state.pose.pose.position.lon) + ", " + \
                      str(self.state.pose.pose.position.alt) + ", " + \
                      str(self.state.pose.pose.position.rel_alt) + ", " + \
                      str(self.state.pose.pose.orientation.x) + ", " + \
                      str(self.state.pose.pose.orientation.y) + ", " + \
                      str(self.state.pose.pose.orientation.z) + ", " + \
                      str(self.state.pose.pose.orientation.w) + ", " + \
                      str(self.state.twist.twist.linear.x) + ", " + \
                      str(self.state.twist.twist.linear.y) + ", " + \
                      str(self.state.twist.twist.linear.z) + ", " + \
                      str(self.state.twist.twist.angular.x) + ", " + \
                      str(self.state.twist.twist.angular.y) + ", " + \
                      str(self.state.twist.twist.angular.z) + ")"

###############################################################################


# Object that maintains the status of all known aircraft in the
# swarm.  Currently, this object just maintains the position and velocity of
# each swarm member as it is broadcast over the network.  Data for each element
# is maintained in a dictionary object with 1 record for each swarm member.
#
# Class member variables:
#   ownID:  ID (integer) of this particular aircraft
#   subSwarmID:  ID (integer) of the subswarm this vehicle is a part of
#   swarmState:  current swarm state of this vehicle (integer)
#   swarmBehavior:  currently active swarm behavior for this vehicle (integer)
#   _baseAlt: Altitude from which rel_alt values are calculated for all AC
#   _swarm: Dictionary of records for individual aircraft in the swarm
#   _swarmPublisher: Object for publishing swarm state to the ROS topic
#   _subSwarmPublisher: Object for publishing subswarm state to the ROS topic
#   _swarmMessage: Container for swarm states to be published to the ROS topic
#   _subSwarmMessage: Container for subswarm states to be published to the ROS topic
#   _lock: Prevents the callback thread modifications at a bad time
#
# Inherited from Nodeable:
#   nodeName:  name of the node to start or node in which the object is
#   timer: ROS rate object that controls the timing loop
#   DBUG_PRINT: set true to force screen debug messages (default FALSE)
#   WARN_PRINT: set false to force screen warning messages (default FALSE) 
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
#        the functionality of this object will be increased accordingly
class SwarmTracker(Nodeable):

    # Class initializer initializes variables, subscribes to required
    # ROS topics, and creates required ros publishers.  Initializer
    # assumes that the object is already running within an initialized
    # ROS node (i.e., the object does not initialize itself as a node).
    # This enables multiple objects to run within a single node if rqd.
    # @param ownID: ID (integer) of this particular aircraft
    # @param nodeName: name of the ROS node for this object
    # @param subswarm: ID (int) of the "subswarm" to which this vehicle belongs
    def __init__(self, ownID, subswarm = 0, nodeName=NODE_BASENAME):
        Nodeable.__init__(self, nodeName)
        self.ownID = ownID
        self.subSwarmID = subswarm
        self.swarmState = 0
        self.swarmBehavior = 0
        self._baseAlt = 0.0
        self._swarm = dict()
        self._swarmPublisher = None
        self._swarmMessage = SwarmStateStamped()
        self._swarmMessage.header.seq = 0
        self._swarmMessage.header.frame_id = "base_footprint"
        self._lock = RLock()
        rospy.set_param('subswarm_id', self.subSwarmID)
#        self.DBUG_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    # Establishes the callbacks for the SwarmTracker object.  The object
    # subscribes to the acs_pose topic for own-aircraft state updates
    # and the recv_pose topic for swarm member updates
    # @param params: list as follows: []
    def callbackSetup(self, params=[]):
        self.createSubscriber("acs_pose", apbrg.Geodometry, \
                              self.updateOwnPose)
        self.createSubscriber("recv_pose", SwarmVehicleState, \
                              self.updateSwarmPose)
        self.createSubscriber("recv_swarm_ctl_state", SwarmControlState, \
                              self.updateSwarmControlState)
        self.createSubscriber("update_subswarm", std_msgs.msg.UInt8, \
                              self.setSubSwarm)
        self.createSubscriber("swarm_state", std_msgs.msg.UInt8, \
                              self.setSwarmState)
        self.createSubscriber("swarm_behavior", std_msgs.msg.UInt8, \
                              self.setSwarmBehavior)


    # Sets up publishers for the SwarmTracker object.  The object publishes
    # SwarmStatus messages to the swarm_uav_states topic
    # @param params: list of parameters (none required for this method)
    def publisherSetup(self, params=[]):
        self._swarmPublisher = \
            self.createPublisher("swarm_uav_states", SwarmStateStamped, 1)


    # Executes one iteration of the timed loop for the SwarmTracker object
    # The loop computes a DR position for each vehicle in the swarm for the
    # current time and publishes a SwarmState message to the ROS topic.
    def executeTimedLoop(self):
        self._lock.acquire()
        self._swarmMessage.header.stamp = rospy.Time.now()
        del self._swarmMessage.swarm[:]    # Clear current message contents

        vKeys = self._swarm.keys()
        for vID in vKeys:
            vehicle = self._swarm[vID]
            timeDiff = self._swarmMessage.header.stamp - \
                       vehicle.state.header.stamp
            if ((self._swarm[vID].isActive) and (timeDiff > MAX_DR_TIME)):
               self._swarm[vID].isActive = False    # Remove vehicle if last report is old
               self.log_warn("Vehicle ID " + str(vID) +\
                             ": no updates--removed from swarm")
            else:
                vehicle.computeDRPose(self._swarmMessage.header.stamp)
                vehicle._stateMsg.state.header.stamp = self._swarmMessage.header.stamp
                vehicle._stateMsg.state.pose = vehicle.drPose.pose
                vehicle._stateMsg.state.twist = vehicle.state.twist
                self._swarmMessage.swarm.append(vehicle._stateMsg)

        self._swarmPublisher.publish(self._swarmMessage)
        self._lock.release()


    #-----------------------------------------
    # ROS Subscriber callbacks for this object
    #-----------------------------------------

    # Updates swarm information for this aircraft when a new pose is
    # publishedHandles new pose information for this aircraft
    # @param poseMsg: Geodometry object with the new pose
    # TODO:  change to include velocity & covariances when they are
    #        included in the msg
    def updateOwnPose(self, stateMsg):
        try:
            if not self.ownID in self._swarm:
                try:
                    self._lock.acquire()
                    self._swarm[self.ownID] = \
                        SwarmElement(self.ownID, self.subSwarmID, apbrg.Geodometry())
                except Exception as ex:
                    self.log_warn("Self update callback error: " + str(ex))
                finally:
                    self._lock.release()
            element = self._swarm[self.ownID]
            element.updateState(stateMsg, self.subSwarmID, self.swarmState, \
                                self.swarmBehavior)
            element.subSwarmID = self.subSwarmID
            newBaseAlt = element.state.pose.pose.position.alt - \
                         element.state.pose.pose.position.rel_alt
            if abs(newBaseAlt - self._baseAlt) > 0.001:
                self._baseAlt = newBaseAlt

        except Exception as ex:
            self.log_warn("Self update callback error: " + str(ex))


    # Updates swarm information for a swarm aircraft when a new pose is published
    # @param poseMsg: SwarmVehicleState object with the new pose
    def updateSwarmPose(self, poseMsg):
        try:
            self._lock.acquire()
            newTime = poseMsg.state.header.stamp
            poseTime = newTime.secs + (newTime.nsecs / float(1e9))
            # Update an existing element if it's already in the dictionary
            if poseMsg.vehicle_id in self._swarm:
                updateElement = self._swarm[poseMsg.vehicle_id]
                elTime = float(updateElement.state.header.stamp.secs) +\
                         float(updateElement.state.header.stamp.nsecs) / float(1e9)
                if poseTime < elTime: return # older than latest data
                # Don't update the swarm state stuff--not in the network msg
                updateElement.updateState(poseMsg.state, poseMsg.subswarm_id, \
                                          updateElement.swarmState, \
                                          updateElement.swarmBehavior)
                updateElement.state.pose.pose.position.rel_alt = \
                    updateElement.state.pose.pose.position.alt - self._baseAlt
            else:  # Create and initialize a new element if this is the first report
                newElement = \
                    SwarmElement(poseMsg.vehicle_id, poseMsg.subswarm_id, \
                                 poseMsg.state, self._baseAlt)
                self._swarm[poseMsg.vehicle_id] = newElement
                self.log_dbug("new aircraft id=" + str(poseMsg.vehicle_id) + " added to swarm")

            element = self._swarm[poseMsg.vehicle_id]
            element.subSwarmID = poseMsg.subswarm_id
        except Exception as ex:
            self.log_warn("Swarm update callback error: " + str(ex))
        finally:
            self._lock.release()


    # Updates the swarm control state of a swarming aircraft (not this one)
    # @param ctlMsg: SwarmControlState message with the new control info
    def updateSwarmControlState(self, ctlMsg):
        try:
            self._lock.acquire()
            if ctlMsg.vehicle_id in self._swarm:
                updateElement = self._swarm[ctlMsg.vehicle_id]
                updateElement.swarmState = ctlMsg.swarm_state
                updateElement.swarmBehavior = ctlMsg.swarm_behavior
        except Exception as ex:
            self.log_warn("Swarm control update callback error: " + str(ex))
        finally:
            self._lock.release()


    # Updates the "subswarm" in which this vehicle is participating
    # @param swarmMsg: message (UInt8) containing the updated swarm ID
    def setSubSwarm(self, swarmMsg):
        self.subSwarmID = swarmMsg.data
        rospy.set_param('subswarm_id', self.subSwarmID)
        self.log_dbug("subswarm set to %d"%swarmMsg.data)


    # Updates the "swarm state" for this vehicle
    # @param swarmMsg: message (UInt8) containing the updated swarm behavior
    def setSwarmState(self, swarmMsg):
        self.swarmState = swarmMsg.data
        self.log_dbug("swarm behavior set to %d"%swarmMsg.data)


    # Updates the "swarm behavior" that is currently active for this vehicle
    # @param swarmMsg: message (UInt8) containing the updated swarm state
    def setSwarmBehavior(self, swarmMsg):
        self.swarmBehavior = swarmMsg.data
        self.log_dbug("swarm state set to %d"%swarmMsg.data)


    #--------------------------------------
    # Class-specific functions
    #--------------------------------------

    # Computes a DR position for each element in the swarm for a specific time
    # @param drTime time (rospy.rostime.Time object) for the DR computation
    def drSwarm(self, drTime):
        for acft in self._swarm:
            self._swarm[acft].updateDRPose(drTime)

