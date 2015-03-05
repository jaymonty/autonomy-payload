#!/usr/bin/env python

#-----------------------------------------------------------------------
# Slate tasks to be run once certain events occur
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
import subprocess

# General ROS imports
import rospy

# Import ROS message and service types
from autopilot_bridge import msg as autopilot_msg
from autopilot_bridge import srv as autopilot_srv
from ap_msgs import msg as payload_msg
from ap_srvs import srv as payload_srv

#-----------------------------------------------------------------------
# Class to encapsulate a task that may have actions at different events

class Task(object):

    def __init__(self, name):
        self._name = name
        self._acid = rospy.get_param('aircraft_id')

    def getName(self):
        return self._name

    '''
    The following methods are the defined events for each task. Define these
    methods in a subclass for any desired events. For each event, the following
    outcomes are allowed:
     - return None - event has no action (nothing is logged)
     - return True - event succeeded (loginfo generated)
     - return False - event failed (logwarn generated)
     - raise Exception - event failed (logwarn generated with exception text)
    '''

    # Called when node first starts
    def on_start(self):
        return None

    # Called on first Status message with real data
    def on_status(self):
        return None

    # Called when throttle is armed
    # NOTE: may occur multiple times
    def on_arm(self):
        return None

    # Called when throttle is disarmed
    # NOTE: may occur multiple times
    def on_disarm(self):
        return None

    # Called when ROS signals node to shut down
    def on_shutdown(self):
        return None

#-----------------------------------------------------------------------
# Task classes

# Start and stop rosbagging
class rosbagTask(Task):

    def __init__(self):
        Task.__init__(self, "rosbag")
        self._folder = "~/bags/"
        self._prefix = "%s%u" % (self._folder, self._acid)
        self._proc = None

    def on_status(self):
        if not rospy.has_param('rosbag_enable') or \
           not rospy.get_param('rosbag_enable'):
            return None
        subprocess.call("ls %s || mkdir %s" % (self._folder, self._folder),
                        shell=True)
        self._proc = subprocess.Popen("rosbag record -a -o " + self._prefix,
                                     shell=True)
        return bool(self._proc.poll() is None)

    def on_shutdown(self):
        if self._proc is None:
            return None
        if self._proc.poll() is None:
            self._proc.send_signal(subprocess.signal.SIGINT)
        return True

# Set params on the autopilot
class paramTask(Task):

    def __init__(self):
        Task.__init__(self, "AP param")
        self._service = "autopilot/param_set"

    def on_status(self):
        rospy.wait_for_service(self._service, 3.0)
        srv = rospy.ServiceProxy(self._service, autopilot_srv.ParamSet)
        res = srv("SYSID_THISMAV", float(self._acid))
        return res.ok

#-----------------------------------------------------------------------
# Task-running and event-handling class

class TaskRunner(object):

    def __init__(self):
        # List of Task instances
        self.tasks = []

        # Default "last" ROS message instances
        self.last_status = autopilot_msg.Status()

        # ROS subscriptions (assumes rospy is initialized)
        self.sub_status = rospy.Subscriber("autopilot/status",
                                           autopilot_msg.Status,
                                           self._cb_status)

        # Register callback for shutdown event
        rospy.on_shutdown(self._cb_shutdown)

    # Perform an event (string name of method) for a given task (Task())
    def _do_event(self, event):
        rospy.loginfo("Task Runner Event: " + event)
        for task in self.tasks:
            header = "Task '%s' Event '%s'" % (task.getName(), event)
            try:
                result = getattr(task, event)()
                if result is None:
                    # No action was taken, nothing to log
                    pass
                elif result is True:
                    rospy.loginfo("%s: Succeeded" % header)
                elif result is False:
                    rospy.logwarn("%s: Failed" % header)
                else:
                    # This shouldn't happen, but we won't log it (for now)
                    pass
            except Exception as ex:
                rospy.logwarn("%s: Raised Exception: %s" % \
                              (header, str(ex.args[0])))

    # NOTE: Most events are performed by the callback that detects them

    # Handle incoming autopilot status messages
    def _cb_status(self, status):
        if self.last_status.header.stamp.secs == 0 and \
           status.header.stamp.secs > 0:
            self._do_event('on_status')

        if not self.last_status.armed and status.armed:
            self._do_event('on_arm')

        if self.last_status.armed and not status.armed:
            self._do_event('on_disarm')

        self.last_status = status

    # Handle rospy shutdown
    def _cb_shutdown(self):
        self._do_event('on_shutdown')

    # Add a Task() to be run
    def add_task(self, task):
        self.tasks.append(task)

    # Run the handler
    # NOTE: This won't return until ROS shuts down
    def run(self):
        # Immediately handle the 'on_start' event
        self._do_event('on_start')

        # Wait until ROS shuts down (callbacks will handle other events)
        rospy.spin()

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # ROS initialization
    rospy.init_node('task_runner')
    
    # Instantiate TaskRunner and Task instances
    runner = TaskRunner()
    runner.add_task(rosbagTask())
    runner.add_task(paramTask())

    # Run the main loop
    runner.run()

