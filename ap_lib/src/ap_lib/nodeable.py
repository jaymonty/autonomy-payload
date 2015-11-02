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


class Nodeable(object):
    ''' Abstract class for an ACS ROS object that can be contained in
    an already established node or run as its own independent node

    Class member variables:
      nodeName:  Name of the node to start or node in which the object is
      timer: ROS rate object that controls the timing loop
      DBUG_PRINT: set true to force screen debug messages (default FALSE)
      INFO_PRINT: set true to force screen info messages (default FALSE)
      WARN_PRINT: set false to force screen warning messages (default FALSE) 

    Class member functions
      createPublisher: set up a publisher using standard naming convention
      createSubscriber: set up a subscriber using standard naming convention
      createService: set up a service using standard naming convention
      runAsNode: run the object as its own ROS node
      log_debug: log an event for debugging purposes
      log_info: log an event as info
      log_warn: log an event as a warning

    "Virtual" methods for inheriting class implementation
      publisherSetup: set up all publishers for the object
      callbackSetup: set up all subscribers for the object
      serviceSetup: set up all services for the object
      executeTimedLoop: run one iteration of the object's timed loop
    '''

    def __init__(self, name):
        ''' Class initializer initializes base class member variables
        @param name:  name of the node that the object is contained in
        '''
        self.nodeName = name
        self.timer = None
        self.DBUG_PRINT = False
        self.INFO_PRINT = False
        self.WARN_PRINT = False


    #--------------------------------------------------------
    # Class-specific methods implementing class functionality
    #--------------------------------------------------------

    def runAsNode(self, rate=10.0):
        ''' Method that is called to run this object as an independent node.
        Registers as a ROS node, calls the setup functions, and runs a
        timing loop for the object's required processing
        @param rate: rate (hz) of the node's timing loop
        '''
        # Initialize ROS node & set up callbacks, services and publishers
        rospy.init_node(self.nodeName)
        self.serviceSetup()
        self.serviceProxySetup()
        self.callbackSetup()
        self.publisherSetup()

        # Set up the timing object and start the timing loop
        self.timer = rospy.Rate(rate)
        self.log_dbug("starting " + self.nodeName)
        print "Starting " + self.nodeName + " node"
        while not rospy.is_shutdown():
            self.timer.sleep()
            self.executeTimedLoop()


    def createPublisher(self, topicName, msgType, queueSize=1, latched=True):
        ''' Method for initializing a ROS publisher object
        Naming convention for the topic is /nodename/topicname where the
        node name is set when the object is instantiated.  The topic name
        is provided as a parameter.
        NOTE:  This method is provided as a convenient mechanism for
               ensuring a standardized topic naming convention at the
               programming level.  Remap as required in the launch file.
        @param topicName: desired name of the topic (no basename)
        @param msgType: type of ROS message to be published to this topic
        @param queueSize: size of published message queue (default 1)
        @param latched: True to latch the publisher (default False)
        @return the created publisher
        '''
        return rospy.Publisher("%s/%s" %(self.nodeName, topicName), msgType, \
                               tcp_nodelay=True, latch=latched, \
                               queue_size=queueSize)


    def createSubscriber(self, topicName, msgType, msgHandler):
        ''' Method for initializing a subscription to a ROS topic
        Naming convention for the topic is /nodename/topicname where the
        node name is set when the object is instantiated.  The topic name
        is provided as a parameter.
        NOTE:  This method is provided as a convenient mechanism for
               ensuring a standardized topic naming convention at the
               programming level.  Remap as required in the launch file.
        @param topicName: desired name of the topic (no basename)
        @param msgType: type of ROS message to be received from this topic
        @param msgHandler: callback function that will handle the message
        @return the created service object
        '''
        self.log_dbug("subscribing to ROS %s/%s topic"\
                      %(self.nodeName, topicName))
        return rospy.Subscriber("%s/%s" %(self.nodeName, topicName), \
                                msgType, msgHandler)


    def createService(self, srvName, srvType, srvHandler):
        ''' Method for initializing a ROS service
        Naming convention for the service is /nodename/servicename where the
        node name is set when the object is instantiated.  The service name
        is provided as a parameter.
        NOTE:  This method is provided as a convenient mechanism for
               ensuring a standardized service naming convention at the
               programming level.  Remap as required in the launch file.
        @param srvName: desired name of the topic (no basename)
        @param srvType: type of ROS service
        @param srvHandler: function that will handle the service
        @return the created service object
        '''
        self.log_dbug("establishing ROS %s/%s service"\
                      %(self.nodeName, srvName))
        return rospy.Service("%s/%s" %(self.nodeName, srvName), \
                             srvType, srvHandler)


    def createServiceProxy(self, srvName, srvType):
        ''' Method for initializing a ROS service proxy
        Naming convention for the service is /nodename/servicename where the
        node name is set when the object is instantiated.  The service name
        is provided as a parameter.
        NOTE:  This method is provided as a convenient mechanism for
               ensuring a standardized service naming convention at the
               programming level.  Remap as required in the launch file.
        @param srvName: desired name of the topic (no basename)
        @param srvType: type of ROS service
        @return the created service proxy object
        '''
        rospy.wait_for_service("%s/%s" %(self.nodeName, srvName))
        self.log_dbug("establishing proxy for ROS %s/%s service"\
                      %(self.nodeName, srvName))
        return rospy.ServiceProxy("%s/%s" %(self.nodeName, srvName), srvType)


    #-----------------------------------------------------
    # "Virtual" methods to be implemented by child classes
    #-----------------------------------------------------

    def serviceSetup(self):
        ''' Virtual method for setting up object's ROS services
        Should be overridden by implementing classes that implement services
        (leave alone if no services provided)
        '''
        pass


    def serviceProxySetup(self):
        ''' Virtual method for setting up object's ROS service proxies
        Should be overridden by implementing classes that require service
        proxies (leave alone if no service proxies are required)
        '''
        pass


    def callbackSetup(self):
        ''' Virtual method for setting up object's ROS topic subscriptions
        Should be overridden by implementing classes that subscribe to
        ROS topics (leave alone if no subscriptions are required)
        '''
        pass


    def publisherSetup(self):
        ''' Virtual method for setting up object's ROS publishers
        Should be overridden by implementing classes that publish messages
        to ROS topics (leave alone if not required)
        '''
        pass


    def executeTimedLoop(self):
        ''' "Virtual" method for one iteration of the object's timed loop
        Should be overridden by implementing classes that perform actions
        every loop iteration (including publishing to ROS topics at a fixed
        rate).  Can be left alone if the object does not actually do any
        internal processing (i.e., functionality via services and callbacks).
        '''
        pass

    #--------------------------------------
    # Logging and message utility functions
    #--------------------------------------

    def log_dbug(self, msg):
        ''' Debug message logging utility function for this object
        '''
        rospy.logdebug(msg)
        if self.DBUG_PRINT:
            print "..DEBUG.. %s: %s" % (self.nodeName, msg)


    def log_info(self, msg):
        ''' Info message logging utility function for this object
        '''
        rospy.loginfo(msg)
        if self.INFO_PRINT:
            print "..INFO.. %s: %s" % (self.nodeName, msg)


    def log_warn(self, msg):
        ''' Warning message logging utility function for this object
        '''
        rospy.logwarn(msg)
        if self.WARN_PRINT:
            print "**WARN** %s: %s" % (self.nodeName, msg)

