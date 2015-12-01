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

DEFAULT_CRASH_TIME = rospy.Duration(6.0)  # Default time for "crash detection"


class SwarmElement(object):
    ''' Data wrapper for a single swarm element object
    Object for maintaining data about a single swarm member.  Uses a ROS
    Geodometry object to represent vehicle pose and velocity.
    Provides some functionality within the class, but access to individual
    data members is still largely through direct access as opposed to member
    functions.  Covariance values are simple defaults for now and aren't
    maintained with any rigor.  Eventually, noise will need to be characterized
    so that the object can be used as part of a Kalman Filter (or EKF).

    Class member variables:
      ID:  aircraft ID (integer) of the particular swarm member
      subSwarmID: subswarm to which this SwarmElement is assigned
      swarmState: the current swarm state for this vehicle
      swarmBehavior: the currently active swarm behavior for this vehicle
      state: aircraft pose and  (Geodometry)
      drPose: computed (DR) position for a future time (Geodometry)
      _stateMsg: container for the swarm state message (SwarmVehicleState)

    Class member functions
      updateState: update the state information with new data
      updateVelocity: update the velocity information with new data
      computeDRPose: computes a predicted pose based on velocity & time elapsed
      getEulerAngles: returns a 3-vector of Euler angles
      getPosition: returns a 3-vector (lat, lon, alt) position
      setPoseFromEulerAngles: sets the quaternion orientation
    '''

    def __init__(self, ownID, subswarm, initState, baseAlt = 0.0):
        ''' Class initializer sets initial values for member variables
        @param ownID: ID of the aircraft for this state vector
        @param subswarm: ID of the subswarm to which this aircraft belongs
        @param initState: Geodometry object with the initial state information
        @param swarmState
        @param baseAlt: MSL altitude from which rel_alt is calculated
        '''
        self.ID = ownID
        self.subSwarmID = subswarm
        self.swarmState = 0
        self.swarmBehavior = 0
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


    def updateState(self, newState, subswarm, swarmState, swarmBehavior):
        ''' Updates the object to contain new state information
         Displacement values are (x, y, z).  Orientation values are a
         quaternion (x, y, z, w).  Linear velocity data is earth-fixed
         meters per second (vx, vy, vz).  Angular velocity is body-fixed
         radians per second (p, q, r)
         @param newState: Geodometry object with the new state information
         @param subswarm: ID of the subswarm to which this swarm member belongs
         @param swarmState: Updated vehicle's current swarm state
         @param swarmBehavior: Updated vehicle's currently active swarm behavior
        '''
        self.subSwarmID = subswarm
        self.swarmState = swarmState
        self.swarmBehavior = swarmBehavior
        self.state = newState
        self.state.header.frame_id = 'base_footprint'
        self._stateMsg.subswarm_id = subswarm
        self._stateMsg.swarm_state = swarmState
        self._stateMsg.swarm_behavior = swarmBehavior


    def computeDRPose(self, drTime):
        ''' Computes a dead reckoned position from the last reported position
        @param drTime time (rospy.rostime.Time object) for the DR computation

        TODO: implement covariance growth functionality
        '''
        poseTimeFloat = float(self.state.header.stamp.secs) +\
                        float(self.state.header.stamp.nsecs) / float(1e9)
        drTimeFloat = float(drTime.secs) + float(drTime.nsecs) / float(1e9)
        dt = drTimeFloat - poseTimeFloat

        # Use fix time, not DR time so the caller knows how old it is
        # and can decide how much (or whether) to trust the DR solution
        self.drPose.header.stamp.secs = self.state.header.stamp.secs
        self.drPose.header.stamp.nsecs = self.state.header.stamp.nsecs

        if dt <= 0.0:  # If DR time in past, use current pose
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

            # Not DRing altitude to avoid "burble" effect causing larger errors
            self.drPose.pose.pose.position.alt = self.state.pose.pose.position.alt
            self.drPose.pose.pose.position.rel_alt = self.state.pose.pose.position.rel_alt
            self.drPose.pose.covariance = self.state.pose.covariance


    def getEulerAngles(self, whichPose=CURRENT_POSE):
        ''' Extracts the Euler angles (roll, pitch, yaw) of a pose
        @param whichPose:  enumeration (int) identifying current, or DR pose
        @return 3-vector containing requested pose Euler angles
        '''
        quaternion = self.state.pose.pose.orientation
        if whichPose == DR_POSE: quaternion = self.drPose.pose.pose.orientation
        elif not whichPose == CURRENT_POSE: return None  # invalid request
        return tf.transformations.euler_from_quaternion(( quaternion.x,\
                                                          quaternion.y,\
                                                          quaternion.z,\
                                                          quaternion.w ))


    def getPosition(self, whichPose=CURRENT_POSE):
        ''' Extracts the position (lat, lon, alt, rel_alt) of a pose
        @param whichPose: enumeration (int) identifying current, or DR pose
        @return 3-vector containing the requested position information
        '''
        pose = self.state.pose.pose.position
        if whichPose == DR_POSE: pose = self.drPose.pose.pose.position
        elif not whichPose == CURRENT_POSE: return None  # invalid request
        return ( pose.lat, pose.lon, pose.alt, pose.rel_alt )


    def setPoseFromEulerAngles(self, roll, pitch, yaw, whichPose=CURRENT_POSE):
        ''' Sets pose orientation from Euler angles (roll, pitch yaw)
        @param roll: Euler angle (radians) for rotation about the X axis
        @param pitch: Euler angle (radians) for rotation about the Y axis
        @param yaw: Euler angle (radians) for rotation about the Z axis
        @param whichPose: enumeration (int) identifying current, or DR pose
        '''
        pose = self.state.pose.pose.orientation
        if whichPose == DR_POSE: pose = self.drPose.pose.pose.orientation
        elif not whichPose == CURRENT_POSE: return  # invalid request
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.x = quat[0]
        pose.y = quat[1]
        pose.z = quat[2]
        pose.w = quat[3]


    def getAsString(self):
        ''' Generates a string representation of the state
        The returned state string will be in the following form:
        "t=####, (Lat, Lon, Alt, Rel_Alt, Qx, Qy, Qz, Qw, Vx, Vy, Vz, p, q, r)"
        @return the requested string form of the state vector
        '''
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

class SwarmTracker(Nodeable):
    ''' Object that maintains the status of all known swarm UAVs
    Currently, this object maintains the position, velocity, subswarm ID,
    swarm state, and waypoint intent for each swarm UAV as it is received
    from the network.  Data for each element is maintained in a dictionary 
    object with 1 record for each swarm member.

    Class member variables:
      ownID:  ID (integer) of this particular aircraft
      subSwarmID:  ID (integer) of the subswarm this vehicle is a part of
      swarmState:  current swarm state of this vehicle (int)
      swarmBehavior:  currently active swarm behavior for this vehicle (int)
      _possible_crash: set of swarm UAVs suspected of crashing (no reports)
      _crash_timeout: max non-reporting time before a UAV is considered crashed
      _baseAlt: Altitude from which rel_alt values are calculated for all AC
      _swarm: Dictionary of records for individual aircraft in the swarm
      _swarmPublisher: Object for publishing swarm state
      _swarmMessage: Container for swarm states to be published
      _lock: Prevents the callback thread modifications at a bad time

    Inherited from Nodeable:
      nodeName:  name of the node to start or node in which the object is
      timer: ROS rate object that controls the timing loop
      DBUG_PRINT: set true to force screen debug messages (default FALSE)
      WARN_PRINT: set false to force screen warning messages (default FALSE) 

    Class methods
      callbackSetup: sets up subscriptions
      publisherSetup: sets up publishers
      executeTimedLoop: executes one loop iteration
      _updateOwnPose: callback for updated own pose receipt
      _updateSwarmPose: callback for updated swarm UAV pose receipt
      _updateSwarmControlState: callback for updated swarm control state receipt
      _setSubSwarm: updates this UAV's subswarm assignment
      _setSwarmState: updates this UAV's swarm state
      _setSwarmBehavior: updates this UAV's current swarm behavior
    '''

    def __init__(self, ownID, subswarm = 0, nodeName=NODE_BASENAME):
        ''' Initializes variables, subscribes to required ROS topics, and
        creates required ros publishers.  Initializer assumes that the object
        is already running within an initialized ROS node (i.e., the object 
        does not initialize itself as a node). This enables multiple objects
        to run within a single node if rqd.
        @param ownID: ID (integer) of this particular aircraft
        @param nodeName: name of the ROS node for this object
        @param subswarm: ID (int) of the "subswarm" to which this vehicle belongs
        '''
        Nodeable.__init__(self, nodeName)
        self.ownID = ownID
        self.subSwarmID = subswarm
        self.swarmState = 0
        self.swarmBehavior = 0
        self._baseAlt = 0.0
        self._swarm = dict()
        self._possible_crash = set()
        self._crash_timeout = DEFAULT_CRASH_TIME
        self._swarmPublisher = None
        self._swarmMessage = SwarmStateStamped()
        self._swarmMessage.header.seq = 0
        self._swarmMessage.header.frame_id = "base_footprint"
        self._lock = RLock()
        rospy.set_param('subswarm_id', self.subSwarmID)

#        self.DBUG_PRINT = True
#        self.INFO_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    def callbackSetup(self):
        ''' Establishes the callbacks for the SwarmTracker object.
        @param params: list as follows: [] (no required parameters)
        '''
        self.createSubscriber("acs_pose", apbrg.Geodometry, \
                              self._updateOwnPose)
        self.createSubscriber("recv_pose", SwarmVehicleState, \
                              self._updateSwarmPose)
        self.createSubscriber("recv_swarm_ctl_state", SwarmControlState, \
                              self._updateSwarmControlState)
        self.createSubscriber("subswarm_id", std_msgs.msg.UInt8, \
                              self._setSubSwarm)
        self.createSubscriber("swarm_state", std_msgs.msg.UInt8, \
                              self._setSwarmState)
        self.createSubscriber("swarm_behavior", std_msgs.msg.UInt8, \
                              self._setSwarmBehavior)


    def publisherSetup(self):
        ''' Sets up publishers for the SwarmTracker object.
        @param params: list as follows: [] (no required parameters)
        '''
        self._swarmPublisher = \
            self.createPublisher("swarm_uav_states", SwarmStateStamped, 1)


    def executeTimedLoop(self):
        ''' Executes one timed-loop iteration for the SwarmTracker object
        The loop computes a DR position for the current time for each UAV
        in the swarm and publishes a SwarmState message to the ROS topic.
        '''
        self._lock.acquire()
        self._swarmMessage.header.stamp = rospy.Time.now()
        self._swarmMessage.crashed_list = list(self._possible_crash)
        del self._swarmMessage.swarm[:]    # Clear current message contents

        vKeys = self._swarm.keys()
        for vID in vKeys:
            vehicle = self._swarm[vID]
            timeDiff = self._swarmMessage.header.stamp - \
                       vehicle.state.header.stamp

            # Half of an Eventually Reliable Crash Detector (ERCD)
            # (i.e., if a UAV dies, we'll eventually realize it reliably)
            if (vID not in self._possible_crash) and \
               (timeDiff > self._crash_timeout):
                self._possible_crash.add(vID)
                self.log_warn("UAV %d possible crash: no updates for %f secs" \
                              %(vID, (float(str(self._crash_timeout))/1e9)))

            vehicle.computeDRPose(self._swarmMessage.header.stamp)
            vehicle._stateMsg.state.header.stamp = vehicle.state.header.stamp
            vehicle._stateMsg.state.pose = vehicle.drPose.pose
            vehicle._stateMsg.state.twist = vehicle.state.twist
            self._swarmMessage.swarm.append(vehicle._stateMsg)

        self._swarmPublisher.publish(self._swarmMessage)
        self._lock.release()


    #-----------------------------------------
    # ROS Subscriber callbacks for this object
    #-----------------------------------------

    def _updateOwnPose(self, stateMsg):
        ''' Updates swarm info for this UAV when a new pose is published
        @param poseMsg: Geodometry object with the new pose
        TODO: add covariances when they are added to the msg
        '''
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


    def _updateSwarmPose(self, poseMsg):
        ''' Updates swarm info for swarm UAVs when new poses are received
        @param poseMsg: Geodometry object with the new pose
        TODO: add covariances when they are added to the msg
        '''
        try:
            self._lock.acquire()
            poseTime = poseMsg.state.header.stamp
            # Update an existing element if it's already in the dictionary
            if poseMsg.vehicle_id in self._swarm:
                updateElement = self._swarm[poseMsg.vehicle_id]
                elTime = updateElement.state.header.stamp
                if poseTime < elTime: return # older than latest data

                # Half of an Eventually Reliable Crash Detector (ERCD)
                # (i.e., if a UAV dies, we'll eventually realize it reliably)
                if poseMsg.vehicle_id in self._possible_crash:
                    t = rospy.Time.now()
                    if (t - elTime) > self._crash_timeout:
                        self._crash_timeout = t - elTime
                    self._possible_crash.remove(poseMsg.vehicle_id)
                    self.log_info("Received update to possibly crashed UAV %d"\
                                  %poseMsg.vehicle_id)

                updateElement.updateState(poseMsg.state, poseMsg.subswarm_id, \
                                          updateElement.swarmState, \
                                          updateElement.swarmBehavior)
                updateElement.state.pose.pose.position.rel_alt = \
                    updateElement.state.pose.pose.position.alt - self._baseAlt

            else: # Create & initialize new element if this is the first report
                newElement = \
                    SwarmElement(poseMsg.vehicle_id, poseMsg.subswarm_id, \
                                 poseMsg.state, self._baseAlt)
                self._swarm[poseMsg.vehicle_id] = newElement
                self.log_dbug("new aircraft id=" + str(poseMsg.vehicle_id) + \
                              " added to swarm")

            element = self._swarm[poseMsg.vehicle_id]
            element.subSwarmID = poseMsg.subswarm_id
        except Exception as ex:
            self.log_warn("Swarm update callback error: " + str(ex))
        finally:
            self._lock.release()


    def _updateSwarmControlState(self, ctlMsg):
        ''' Updates swarm control state of a swarming aircraft (not this one)
        @param ctlMsg: SwarmControlState message with the new control info
        '''
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


    def _setSubSwarm(self, swarmMsg):
        ''' Updates the "subswarm" in which this vehicle is participating
        @param swarmMsg: message (UInt8) containing the updated swarm ID
        '''
        self.subSwarmID = swarmMsg.data
        rospy.set_param('subswarm_id', self.subSwarmID)
        self.log_dbug("subswarm set to %d"%swarmMsg.data)


    def _setSwarmState(self, swarmMsg):
        ''' Updates the "swarm state" for this vehicle
        @param swarmMsg: message (UInt8) containing the updated swarm behavior
        '''
        self.swarmState = swarmMsg.data
        self.log_dbug("swarm behavior set to %d"%swarmMsg.data)


    def _setSwarmBehavior(self, swarmMsg):
        ''' Updates the "swarm behavior" currently active for this vehicle
        @param swarmMsg: message (UInt8) containing the updated swarm state
        '''
        self.swarmBehavior = swarmMsg.data
        self.log_dbug("swarm state set to %d"%swarmMsg.data)

