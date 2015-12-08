#!/usr/bin/env python

#-----------------------------------------------------------------------
# Flight safety and aircraft health node
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports


# General ROS imports
import rospy
from rosgraph import masterapi

# Import ROS message and service types
from std_msgs import msg as stdmsg
from autopilot_bridge import msg as apmsg
from ap_msgs import msg as ap_msgs
from ap_srvs import srv as ap_srvs

#-----------------------------------------------------------------------
# SafetyMonitor class

class SafetyMonitor(object):

    def __init__(self):
        # Internal variables
        self._health_state = None       # Last-known health state
        self._health_counter = 0        # Counter of published heartbeats
        self._set_state = True          # Commanded health state
        self._hb_last_msg = None        # Last ground heartbeat
        self._hb_last_time = 0.0
        self._hb_interval = 10.0
        self._ap_last_msg = None        # Last autopilot pose
        self._ap_last_time = 0.0
        self._ap_interval = 3.0
        self._mapi = masterapi.Master('safety') # ROS Master API
        self._mapi_nodes = ['autopilot',
                            'network',
                            'swarm_tracker',
                            'task_runner']
        self._mapi_namespace = rospy.get_namespace()
        self._mapi_last_result = True
        self._mapi_last_time = 0.0
        self._mapi_interval = 5.0

        # ROS objects
        bname = rospy.get_name()
        self._pub_heartbeat = rospy.Publisher(bname+"/heartbeat",
                                              apmsg.Heartbeat,
                                              queue_size=1)
        self._pub_healthy = rospy.Publisher(bname+"/healthy",
                                            stdmsg.Bool,
                                            queue_size=1,
                                            latch=True)
        self._sub_heartbeat = rospy.Subscriber(bname+"/gnd_heartbeat",
                                               apmsg.Heartbeat,
                                               self._sub_ground_heartbeat)
        self._sub_pose = rospy.Subscriber(bname+"/ap_pose",
                                          apmsg.Geodometry,
                                          self._sub_autopilot_pose)
        self._srv_state = rospy.Service(bname+"/set_health_state",
                                        ap_srvs.SetBoolean,
                                        self._srv_set_state)

    #-------------------------------------------------------------------
    # Main loop

    def run(self):
        ''' Main loop '''
        COUNTER_MAX = 2**32
        hb = apmsg.Heartbeat()
        healthy = stdmsg.Bool()
        r = rospy.Rate(1.0)

        while not rospy.is_shutdown():
            # Check all safety conditions
            ok = self._is_ok()

            # Publish a heartbeat, only if all is well
            if ok:
                self._health_counter = (self._health_counter + 1) % COUNTER_MAX
                hb.counter = self._health_counter
                self._pub_heartbeat.publish(hb)

            # If health has changed, publish to latched topic
            if ok != self._health_state:
                self._health_state = ok
                healthy.data = ok
                self._pub_healthy.publish(healthy)

            # Sleep so ROS subscribers and services can run
            r.sleep()

    def _is_ok(self):
        ''' Return boolean indicating if system is "ok" '''
        ok = True
        for check in [self._chk_set_state,
                      self._chk_nodes_alive,
                      #self._chk_ground_heartbeat,  # TODO No HB in SITL yet
                      self._chk_autopilot_pose]:
            try:
                if not check():
                    rospy.logdebug("SAFETY: check %s failed" % \
                                   check.__name__)
                    ok = False
            except Exception as ex:
                rospy.logwarn("SAFETY: check %s error: %s" % \
                              (check.__name__, str(ex)))
                ok = False
        return ok

    #-------------------------------------------------------------------
    # ROS subscriptions and services

    def _sub_ground_heartbeat(self, msg):
        ''' Callback for received ground heartbeat '''
        self._hb_last_msg = msg
        self._hb_last_time = rospy.get_time()

    def _sub_autopilot_pose(self, msg):
        ''' Callback for autopilot pose message '''
        self._ap_last_msg = msg
        self._ap_last_time = rospy.get_time()

    def _srv_set_state(self, req):
        ''' Callback for setting explicit health state '''
        self._set_state = bool(req.enable)
        return 1

    #-------------------------------------------------------------------
    # Checking methods

    def _chk_set_state(self):
        ''' Check whether health state was explicitly commanded '''
        return self._set_state

    def _chk_nodes_alive(self):
        ''' Check if critical ROS nodes are alive '''
        t = rospy.get_time()
        if t < self._mapi_last_time + self._mapi_interval:
            return self._mapi_last_result

        self._mapi_last_result = True
        self._mapi_last_time = t

        for node in self._mapi_nodes:
            name = self._mapi_namespace + node
            try:
                res = self._mapi.lookupNode(name)
                if not isinstance(res, str): raise Exception('')
            except:
                self._mapi_last_result = False
                break

        return self._mapi_last_result

    def _chk_ground_heartbeat(self):
        ''' Check if ground heartbeat has been received recently '''
        if self._hb_last_msg is None:
            return False

        t = rospy.get_time()
        if t > self._hb_last_time + self._hb_interval:
            return False

        return True

    def _chk_autopilot_pose(self):
        ''' Check if autopilot pose has been received recently '''
        if self._ap_last_msg is None:
            return False

        t = rospy.get_time()
        if t > self._ap_last_time + self._ap_interval:
            return False

        return True


#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # ROS initialization
    rospy.init_node('safety')
    
    # Set up monitor object
    monitor = SafetyMonitor()
    
    # Wrap monitor run loop in try block, never let it die
    print "\nStarting safety monitor loop...\n"
    while not rospy.is_shutdown():
        try:
            monitor.run()
        except Exception as ex:
            rospy.logwarn("SAFETY: loop error: " + str(ex))

