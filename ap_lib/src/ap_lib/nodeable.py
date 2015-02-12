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
#   timer: ROS rate object that controls the timing loop
#   DBUG_PRINT: set true to force screen debug messages (default FALSE)
#   WARN_PRINT: set false to force screen warning messages (default FALSE) 
#
# Class member functions
#   createPublisher: set up one ROS publisher using a standardized naming convention
#   createSubscriber: set up one ROS subscriber using a standardized naming convention
#   createService: set up one ROS service using a standardized naming convention
#   runAsNode: run the object as its own ROS node
#   log_debug: log an event for debugging purposes
#   log_warn: log an event as a warning
#
# "Virtual" methods for inheriting class implementation
#   publisherSetup: set up all publishers for the object
#   callbackSetup: set up all subscribers for the object
#   serviceSetup: set up all services for the object
#   executeTimedLoop: run one iteration of the object's timed loop
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
    # @param serviceParams: list of arbitrary parameters to set up services
    # @param callbackParams: list of arbitrary parameters to set up callbacks
    # @param publisherParams: list of arbitrary parameters to set up publishers
    def runAsNode(self, rate=10.0, serviceParams=[], callbackParams=[], \
                  publisherParams=[]):
        # Initialize ROS node & set up callbacks, services and publishers
        rospy.init_node(self.nodeName)
        self.serviceSetup(serviceParams)
        self.serviceProxySetup([])
        self.callbackSetup(callbackParams)
        self.publisherSetup(publisherParams)

        # Set up the timing object and start the timing loop
        self.timer = rospy.Rate(rate)
        self.log_dbug("starting " + self.nodeName)
        print "Starting " + self.nodeName + " node"
        while not rospy.is_shutdown():
            self.timer.sleep()
            self.executeTimedLoop()


    # Method for initializing a ROS publisher object
    # Naming convention for the topic is /nodename/topicname where the
    # node name is set when the object is instantiated.  The topic name
    # is provided as a parameter.
    # NOTE:  This method is provided as a convenient mechanism for
    #        ensuring a standardized topic naming convention at the
    #        programming level.  Remap as required in the launch file.
    # @param topicName: desired name of the topic (no basename)
    # @param msgType: type of ROS message to be published to this topic
    # @param queueSize: size of published message queue (default 1)
    # @param latched: True to latch the publisher (default False)
    # @return the created publisher
    def createPublisher(self, topicName, msgType, queueSize=1, latched=True):
        return rospy.Publisher("%s/%s" %(self.nodeName, topicName), msgType, \
                               tcp_nodelay=True, latch=latched, \
                               queue_size=queueSize)


    # Method for initializing a subscription to a ROS topic
    # Naming convention for the topic is /nodename/topicname where the
    # node name is set when the object is instantiated.  The topic name
    # is provided as a parameter.
    # NOTE:  This method is provided as a convenient mechanism for
    #        ensuring a standardized topic naming convention at the
    #        programming level.  Remap as required in the launch file.
    # @param topicName: desired name of the topic (no basename)
    # @param msgType: type of ROS message to be received from this topic
    # @param msgHandler: callback function that will handle the message
    # @return the created service object
    def createSubscriber(self, topicName, msgType, msgHandler):
        return rospy.Subscriber("%s/%s" %(self.nodeName, topicName), \
                                msgType, msgHandler)


    # Method for initializing a ROS service
    # Naming convention for the service is /nodename/servicename where the
    # node name is set when the object is instantiated.  The service name
    # is provided as a parameter.
    # NOTE:  This method is provided as a convenient mechanism for
    #        ensuring a standardized service naming convention at the
    #        programming level.  Remap as required in the launch file.
    # @param srvName: desired name of the topic (no basename)
    # @param srvType: type of ROS service
    # @param srvHandler: function that will handle the service
    # @return the created service object
    def createService(self, srvName, srvType, srvHandler):
        return rospy.Service("%s/%s" %(self.nodeName, srvName), \
                             srvType, srvHandler)


    # Method for initializing a ROS service proxy
    # Naming convention for the service is /nodename/servicename where the
    # node name is set when the object is instantiated.  The service name
    # is provided as a parameter.
    # NOTE:  This method is provided as a convenient mechanism for
    #        ensuring a standardized service naming convention at the
    #        programming level.  Remap as required in the launch file.
    # @param srvName: desired name of the topic (no basename)
    # @param srvType: type of ROS service
    # @return the created service proxy object
    def createServiceProxy(self, srvName, srvType):
        return rospy.ServiceProxy("%s/%s" %(self.nodeName, srvName), srvType)


    #-----------------------------------------------------
    # "Virtual" methods to be implemented by child classes
    #-----------------------------------------------------

    # Virtual method for setting up services that this object will
    # be providing.  Should be overridden by implementing classes
    # that implement services (leave alone if no services provided)
    # @param params: list of arbitrary parameters for implementing class use
    def serviceSetup(self, params=[]):
        pass


    # Virtual method for setting up service proxies that this object will
    # utilize.  Should be overridden by implementing classes
    # that require services (leave alone if no services utilized)
    # @param params: list of arbitrary parameters for implementing class use
    def serviceProxySetup(self, params=[]):
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

