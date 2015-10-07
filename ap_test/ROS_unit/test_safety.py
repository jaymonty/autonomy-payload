#!/usr/bin/env python
PKG='ap_test'

import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin
                                    #But it is need for command line roslaunch 
import rospy

import sys, time, unittest

from autopilot_bridge import msg as apmsg
from ap_srvs import srv as ap_srvs

class TestSafetyNode(unittest.TestCase):
    def hb_callback(self, data):
        self.last_hb = data.counter

    #The setUp method is run before EACH test_* method is run.
    def setUp(self):
        rospy.init_node('test_safety')

        #May only need to do this in __init__, but as long as re-setting up
        #the proxy for each unit test doesn't cause noticeable slowdown
        #we'll keep it in the setUp method.        
        self.set_health_state = rospy.ServiceProxy('/safety/set_health_state',
                                                   ap_srvs.SetBoolean)

        self.subscriber = rospy.Subscriber("safety/heartbeat", apmsg.Heartbeat, self.hb_callback)
        self.last_hb = -1

    def test_svc_set_health(self):
        #ensure we can set an unset health state and also test
        #a couple error cases. T/F doesn't really have any edge cases. :)
        self.assertTrue(self.set_health_state(True), "SetHealthTrue")
        self.assertTrue(self.set_health_state(False), "SetHealtFalse")
        self.assertTrue(self.set_health_state(1), "SetHealthInt")
        #TODO: fix the node so this test passes!
        #self.assertTrue(self.set_health_state('t'), "SetHealthBadStr")

    def test_pub_heartbeat(self):
        #need to be slow for the heartbeats to increment:
        self.r = rospy.Rate(0.8)

        #ensure the hearbeat increments as expected when healthy
        self.set_health_state(True)
        self.r.sleep()
        prev_hb = self.last_hb
        self.r.sleep()        
        self.assertGreater(self.last_hb, prev_hb, "HearbeatIncrement")
        
        #ensure the heartbeat does not increment when not healthy
        self.set_health_state(False)
        prev_hb = self.last_hb
        self.r.sleep()
        self.assertEqual(self.last_hb, prev_hb, "HeartbeatNoIncrement")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_safety', TestSafetyNode)
    
