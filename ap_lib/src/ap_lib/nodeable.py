#!/usr/bin/env python

#-----------------------------------------------------------------------
# Nodeable
# Duane Davis, 2014
#
# Defines an abstract class for wrapping ACS project classes that
# can be incorporated into other classes or run as independent nodes
#-----------------------------------------------------------------------

# ROS imports
import rospy
from rospy import rostime


# Abstract object for wrapping an ACS ROS object that can be contained in
# an already established node or run as its own independent node
#
# Class member variables:
#   nodeName:  Name of the node to start or node in which the object is
#   DBUG_PRINT: set true to force screen debug messages (default FALSE)
#   WARN_PRINT: set false to force screen warning messages (default FALSE) 
#   timer: ROS rate object that controls the timing loop
class Nodeable(object):

    # Class initializer initializes base class member variables
    # @param name:  name of the node that the object is contained in
    def __init__(self, name):
        self.nodeName = name
        self.timer = None
        self.DBUG_PRINT = False
        self.WARN_PRINT = False


    #--------------------------------------------------------
    # Class-specific methods implementing class functionality
    #--------------------------------------------------------

    # Method that is called to run this object as an independent node.
    # Registers as a ROS node, calls the setup functions, and runs a
    # timing loop for the object's required processing
    # @param rate: rate (hz) of the node's timing loop
    # @param 
    def runAsNode(self, rate=10.0, serviceParams=[], callbackParams=[], \
                  publisherParams=[]):
        # Initialize ROS node & set up callbacks, services and publishers
        rospy.init_node(self.nodeName)
        self.serviceSetup(serviceParams)
        self.callbackSetup(callbackParams)
        self.publisherSetup(publisherParams)

        # Set up the timing object and start the timing loop
        self.log_dbug("starting " + self.nodeName)
        self.timer = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.timer.sleep()
            self.executeTimedLoop()


    # Virtual method for setting up services that this object will
    # be providing.  Should be overridden by implementing classes
    # that implement services (leave alone if no services provided)
    # @param params: list of arbitrary parameters for implementing class use
    def serviceSetup(self, params=[]):
        pass


    # Virtual method for setting up callbacks for topics to which
    # the implementing class will subscribe.   that this object will
    # be providing.  Should be overridden by implementing classes
    # that subscribe to ROS topics (leave alone if no subscriptions)
    # @param params: list of arbitrary parameters for implementing class use
    def callbackSetup(self, params=[]):
        pass


    # Virtual method for setting up publishers for topics to which this
    # object will publish information.  Should be overridden by implementing
    # classes that publish to ROS topics (leave alone if no publishers)
    # @param params: list of arbitrary parameters for implementing class use
    def publisherSetup(self, params=[]):
        pass


    # Virtual method for the objects internal processing for each iteration
    # of the timing loop.  Should be overridden by implementing classes
    # that perform actions every loop iteration (including publishing to
    # ROS topics at a fixed rate).  Can be left alone if the object does not
    # actually do any internal processing (i.e., all functionality achieved
    # through services and callbacks).
    def executeTimedLoop(self):
        pass


    #--------------------------------------
    # Logging and message utility functions
    #--------------------------------------

    # Logging utility function for this object
    def log_dbug(self, msg):
        rospy.logdebug(msg)
        if self.DBUG_PRINT:
            print "..DEBUG.. %s: %s" % (self.nodeName, msg)


    # Warning utility function for this object
    def log_warn(self, msg):
        rospy.logwarn(msg)
        if self.WARN_PRINT:
            print "**WARN** %s: %s" % (self.nodeName, msg)

