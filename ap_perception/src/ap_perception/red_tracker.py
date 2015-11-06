#!/usr/bin/env python

#-----------------------------------------------------------------------
# Red Tracker
# Duane Davis, 2015
#
# Defines an object that keeps track of all "red" UAVs
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
NODE_BASENAME = 'red_tracker'

# Some constants to make functions more "reusable"
CURRENT_POSE = 0
DR_POSE = 1

MAX_DR_TIME = rospy.Duration(5.0)  # Max time for a DR position update


class RedRecord(object):
    ''' Data wrapper for a single red UAVt object

    Object for maintaining data about a single "red" UAV.  Uses a ROS
    Geodometry object to represent vehicle pose and velocity.
    Provides some functionality within the class, but access to individual
    data members is still largely through direct access as opposed to member
    functions.  Covariance values are simple defaults for now and aren't
    maintained with any rigor.  Eventually, noise will need to be characterized
    so that the object can be used as part of a Kalman Filter (or EKF).

    Class member variables:
      ID:  aircraft ID (integer) of the particular red UAV
      isActive: indicates that the vehicle is active (recent update)
      state: aircraft pose and  (Geodometry)
      drPose: computed (DR) position for a future time (Geodometry)
      _stateMsg: container for the red UAV state message (RedVehicleState)

    Class member functions
      updateState: update the state information with new data
      updateVelocity: update the velocity information with new data
      computeDRPose: computes a predicted pose based on velocity & time elapsed
      getEulerAngles: returns a 3-vector of Euler angles
      getPosition: returns a 3-vector (lat, lon, alt) position
      setPoseFromEulerAngles: sets the quaternion orientation
    '''

    def __init__(self, uavID, initState, baseAlt = 0.0):
        ''' Class initializer sets initial values for member variables
        @param uavID: ID of the aircraft for this state vector
        @param initState: Geodometry object with the initial state information
        @param baseAlt: MSL altitude from which rel_alt is calculated
        '''
        self.ID = uavID
        self.isActive = True
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
        self._stateMsg = RedVehicleState()
        self._stateMsg.vehicle_id = self.ID
        self._stateMsg.state.pose.pose.position.using_alt = True
        self._stateMsg.state.pose.pose.position.using_rel_alt = True


    def updateState(self, newState):
        ''' Updates the object to contain new state information
         Displacement values are (x, y, z).  Orientation values are a
         quaternion (x, y, z, w).  Linear velocity data is earth-fixed
         meters per second (vx, vy, vz).  Angular velocity is body-fixed
         radians per second (p, q, r)
         @param newState: Geodometry object with the new state information
        '''
        self.isActive = True
        self.state = newState
        self.state.header.frame_id = 'base_footprint'


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

class RedTracker(Nodeable):
    ''' Object that maintains the status of all known red UAVs
    Currently, this object maintains the position and velocity for each known
    red UAV as it is received from the network.  Data is maintained in a
    dictionary object with 1 record for each red UAV.

    Class member variables:
      ownID:  ID (integer) of this particular aircraft
      _baseAlt: Altitude from which rel_alt values are calculated for all AC
      _reds: Dictionary of records for individual aircraft in the swarm
      _redsPublisher: Object for publishing red swarm state
      _redsMessage: Container for red swarm states to be published
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
      _receivedOwnPose: callback for updated own pose receipt
      _updateRedPose: callback for updated red UAV pose receipt
    '''

    def __init__(self, ownID, nodeName=NODE_BASENAME):
        ''' Initializes variables, subscribes to required ROS topics, and
        creates required ros publishers.  Initializer assumes that the object
        is already running within an initialized ROS node (i.e., the object 
        does not initialize itself as a node). This enables multiple objects
        to run within a single node if rqd.
        @param ownID: ID (integer) of this particular aircraft
        @param nodeName: name of the ROS node for this object
        '''
        Nodeable.__init__(self, nodeName)
        self.ownID = ownID
        self._baseAlt = 0.0
        self._reds = dict()
        self._redsPublisher = None
        self._redsMessage = RedSwarmStateStamped()
        self._redsMessage.header.seq = 0
        self._redsMessage.header.frame_id = "base_footprint"
        self._lock = RLock()

#        self.DBUG_PRINT = True
#        self.INFO_PRINT = True
#        self.WARN_PRINT = True


    #-------------------------------------------------
    # Implementation of parent class virtual functions
    #-------------------------------------------------

    def callbackSetup(self):
        ''' Establishes the callbacks for the RedTracker object.
        '''
        self.createSubscriber("acs_pose", apbrg.Geodometry, \
                              self.receivedOwnPose)
        self.createSubscriber("recv_red_pose", RedVehicleState, \
                              self._updateRedPose)


    def publisherSetup(self):
        ''' Sets up publishers for the RedTracker object.
        '''
        self._redsPublisher = \
            self.createPublisher("red_uav_states", RedSwarmStateStamped, 1)


    def executeTimedLoop(self):
        ''' Executes one timed-loop iteration for the RedTracker object
        The loop computes a DR position for the current time for each red UAV
        and publishes a RedSwarmState message to the ROS topic.
        '''
        with self._lock:
            self._redsMessage.header.stamp = rospy.Time.now()
            del self._redsMessage.reds[:]    # Clear current message contents

            vKeys = self._reds.keys()
            for vID in vKeys:
                vehicle = self._reds[vID]
                timeDiff = self._redsMessage.header.stamp - \
                           vehicle.state.header.stamp
                if ((self._reds[vID].isActive) and (timeDiff > MAX_DR_TIME)):
                   self._reds[vID].isActive = False 
                   self.log_warn("UAV ID %d: no updates for %s seconds--use with caution"\
                                 %(vID, str(MAX_DR_TIME/1e9)))

                vehicle.computeDRPose(self._redsMessage.header.stamp)
                vehicle._stateMsg.state.header.stamp = \
                    vehicle.state.header.stamp
                vehicle._stateMsg.state.pose = vehicle.drPose.pose
                vehicle._stateMsg.state.twist = vehicle.state.twist
                self._redsMessage.reds.append(vehicle._stateMsg)

            self._redsPublisher.publish(self._redsMessage)


    #-----------------------------------------
    # ROS Subscriber callbacks for this object
    #-----------------------------------------

    def receivedOwnPose(self, stateMsg):
        ''' Uses the acs_pose message to update the base altitude
        @param stateMsg: Geodometry object with the new pose
        '''
        newBaseAlt = stateMsg.pose.pose.position.alt - \
                     stateMsg.pose.pose.position.rel_alt
        if abs(newBaseAlt - self._baseAlt) > 0.001:
            self._baseAlt = newBaseAlt


    def _updateRedPose(self, poseMsg):
        ''' Updates info for red UAVs when new poses are received
        @param poseMsg: Geodometry object with the new pose
        TODO: add covariances when they are added to the msg
        '''
        try:
            self._lock.acquire()
            newTime = poseMsg.state.header.stamp
            poseTime = newTime.secs + (newTime.nsecs / float(1e9))
            # Update an existing element if it's already in the dictionary
            if poseMsg.vehicle_id in self._reds:
                update_record = self._reds[poseMsg.vehicle_id]
                elTime = \
                    float(update_record.state.header.stamp.secs) +\
                    float(update_record.state.header.stamp.nsecs) / float(1e9)
                if poseTime < elTime: return # older than latest data
                if not update_record.isActive:
                    self.log_warn("received update from previously inactive red UAV %d"\
                                  %poseMsg.vehicle_id)
                update_record.updateState(poseMsg.state)
                update_record.state.pose.pose.position.rel_alt = \
                    update_record.state.pose.pose.position.alt - self._baseAlt

            else: # Create & initialize new record if this is the first report
                new_record = \
                    RedRecord(poseMsg.vehicle_id, poseMsg.state, self._baseAlt)
                self._reds[poseMsg.vehicle_id] = new_record
                self.log_dbug("new aircraft id=%d added to red tracker"\
                              %poseMsg.vehicle_id)

            element = self._reds[poseMsg.vehicle_id]
        except Exception as ex:
            self.log_warn("Red pose update callback error: %s" %str(ex))
        finally:
            self._lock.release()


#-----------------------------------------------------------------------
# Main code

if __name__ == '__main__':
    ownAC = int(rospy.get_param("aircraft_id"))
    red_tracker = RedTracker(ownAC, NODE_BASENAME)
    red_tracker.runAsNode(10.0)


