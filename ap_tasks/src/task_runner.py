#!/usr/bin/env python

#-----------------------------------------------------------------------
# Slate tasks to be run once certain events occur
# Mike Clement, 2014
#
# Some general info should go here!!

#-----------------------------------------------------------------------
# Import a bunch of libraries

# Standard Python imports
import os
import subprocess
from threading import RLock
import time

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

    def _wait(self, srv):
        rospy.wait_for_service(srv, 3.0)

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

    # Called when flight_ready flag is set
    # NOTE: may occur multiple times
    def on_ready(self):
        return None

    # Called when flight_ready flag is UNset
    # NOTE: may occur multiple times
    def on_not_ready(self):
        return None

    # Called when ROS signals node to shut down
    def on_shutdown(self):
        return None

#-----------------------------------------------------------------------
# Task-running and event-handling class

class TaskRunner(object):

    def __init__(self):
        # Lock to prevent multiple tasks from running at once
        self.lock = RLock()

        # List of Task instances
        self.tasks = []

        # Default "last" ROS message/param instances
        self.last_status = autopilot_msg.Status()
        self.last_flightready = False

        # Flags telling if one-time events have happened yet
        self.done_on_status = False

        # ROS subscriptions (assumes rospy is initialized)
        self.sub_status = rospy.Subscriber("autopilot/status",
                                           autopilot_msg.Status,
                                           self._cb_status)

    def _log(self, event, task, start, success, info=''):
        end = time.time()
        s = "Task Runner: Event '%s' Task '%s' Time %0.1fs: " % \
            (event, task.getName(), end - start)
        si = ""
        if info:
            si = " (%s)" % info
        if success:
            rospy.loginfo(s + "Succeeded" + si)
        else:
            rospy.logwarn(s + "Failed" + si)

    # Perform an event (string name of method) for a given task (Task())
    def _do_event(self, event):
        with self.lock:
            rospy.loginfo("Task Runner: Starting Event '%s'" % event)
            for task in self.tasks:
                start = time.time()
                try:
                    result = getattr(task, event)()
                    if result is None:
                        # No action was taken, nothing to log
                        continue
                    self._log(event, task, start, bool(result))
                except Exception as ex:
                    self._log(event, task, start, False, str(ex))
            rospy.loginfo("Task Runner: Completed Event '%s'" % event)

    # NOTE: Most events are performed by the callback that detects them

    # Handle incoming autopilot status messages
    def _cb_status(self, status):
        if not self.done_on_status and status.header.stamp.secs > 0:
            self._do_event('on_status')
            self.done_on_status = True

        if not self.last_status.armed and status.armed:
            self._do_event('on_arm')

        if self.last_status.armed and not status.armed:
            self._do_event('on_disarm')

        self.last_status = status

    # Add a Task() to be run
    def add_task(self, task):
        self.tasks.append(task)

    # Run the handler
    # NOTE: This won't return until ROS shuts down
    def run(self):
        # Immediately handle the 'on_start' event
        self._do_event('on_start')

        # Wait until ROS shuts down (callbacks will handle other events)
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            # Check flight ready status
            flightready = rospy.has_param("flight_ready") and \
                          rospy.get_param("flight_ready")
            if flightready and not self.last_flightready:
                self._do_event('on_ready')
            if not flightready and self.last_flightready:
                self._do_event('on_not_ready')
            self.last_flightready = flightready

            # Sleep for a bit, let subscribers run
            r.sleep()

        # Handle the shutdown event
        self._do_event('on_shutdown')

#-----------------------------------------------------------------------
# Task classes
# NOTE: This should be the only part to be customized

# Start and stop rosbagging
class RosbagTask(Task):

    def __init__(self):
        Task.__init__(self, "Rosbag")
        self._folder = os.path.expanduser("~/bags/")
        self._excl = "\"(.*)(swarm_tracker/swarm_uav_states|network/recv_pose)\""
        self._prefix = "%s%u" % (self._folder, self._acid)
        self._cmd = "rosbag record -a -x " + self._excl + " -o " + self._prefix
        self._proc = None

    def on_status(self):
        # Build the folder if it doesn't exist
        try:
            os.stat(self._folder)
        except:
            os.mkdir(self._folder)

        # Launch the process
        self._proc = subprocess.Popen(self._cmd, shell=True)
        return bool(self._proc.poll() is None)

    def on_shutdown(self):
        if self._proc is None:
            return None
        if self._proc.poll() is None:
            self._proc.send_signal(subprocess.signal.SIGINT)
        return True

# Set ID on the autopilot
class SetIDTask(Task):

    def __init__(self):
        Task.__init__(self, "AP Set ID")
        self._srv = "autopilot/fpr_param_set"

    def on_status(self):
        self._wait(self._srv)
        srv = rospy.ServiceProxy(self._srv, autopilot_srv.ParamSet)
        res = srv("SYSID_THISMAV", float(self._acid))
        return res.ok

# Increase/decrease power based on arming state
class TxPowerTask(Task):

    def __init__(self):
        Task.__init__(self, "Tx Power")

    def _set(self, level):
        res = subprocess.call("sudo iwconfig wlan0 txpower %u" % level,
                              shell=True)
        return bool(res == 0)

    def on_start(self):
        return self._set(10)

    def on_ready(self):
        return self._set(20)

    def on_not_ready(self):
        return self._set(10)

# Fetch "blessed" configs
class FetchConfigTask(Task):

    def __init__(self):
        Task.__init__(self, "Fetch Configs")
        self._folder = os.path.expanduser("~/blessed/")
        self._id_str = "%03u" % int(self._acid)

    def on_status(self):
        # Build the folder if it doesn't exist
        try:
            os.stat(self._folder)
        except:
            os.mkdir(self._folder)

        # Delete files that MUST be refreshed
        for f in ['fence', 'param', 'rally', 'wp',
                  'rally.template', 'wp.template']:
            try:
                os.remove(self._folder + f)
            except:
                pass

        # Fetch any needed files
        result = True
        for f in ['fence', 'param', 'rally', 'wp',
                  'rally.template', 'wp.template']:
            # Check if file already exists (if not deleted above)
            try:
                s = os.stat(self._folder + f)
                if s.st_size > 0: continue
            except:
                pass

            # If not, fetch it
            rospy.loginfo("Fetching %s from server ..." % f)
            for i in range(3):
                if rospy.is_shutdown(): return False
                try:
                    cmd = "wget -q -T 20 -O %s%s http://192.168.2.1/%s/%s" % \
                          (self._folder, f, self._id_str, f)
                    res = subprocess.call(cmd, shell=True)
                    if res == 0: break
                    time.sleep(3)
                except Exception as ex:
                    rospy.logwarn("Exception fetching %s: %s" % \
                                  (f, str(ex.args[0])))
            if res != 0:
                rospy.logwarn("Failed to fetch " + f)
                result = False
        return result

# Verify "blessed" configs
class VerifyConfigTask(Task):

    def __init__(self):
        Task.__init__(self, "Verify Configs")
        self._folder = os.path.expanduser("~/blessed/")

    def _try_verify(self, f):
        try:
            s_name = "autopilot/load_" + f
            f_name = self._folder + f
            self._wait(s_name)
            srv = rospy.ServiceProxy(s_name, autopilot_srv.FileLoad)
            res = srv(f_name)
            return res.ok
        except Exception as ex:
            rospy.logwarn("Error verifying %s: %s" % (f, str(ex.args[0])))

    def on_status(self):
        result = True
        for f in ['rally', 'wp', 'fence', 'param']:
            rospy.set_param("ok_" + f, False)

            # If file doesn't exist, skip it
            try:
                s = os.stat(self._folder + f)
                if s.st_size == 0: raise Exception('')
            except:
                rospy.logwarn("Skipping verify of missing file " + f)
                continue

            for i in range(3):
                if rospy.is_shutdown(): return False
                res = self._try_verify(f)
                if res: break
                time.sleep(1)
            if res:
                rospy.set_param("ok_" + f, True)
            else:
                rospy.logwarn("Failed to verify " + f)
                result = False
        return result

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # ROS initialization
    rospy.init_node('task_runner')
    
    # Instantiate TaskRunner and Task instances
    runner = TaskRunner()
    runner.add_task(SetIDTask())

    if rospy.has_param('network_device') and \
       'wlan' in rospy.get_param('network_device'):
        runner.add_task(TxPowerTask())

    if rospy.has_param('verify_enable') and rospy.get_param('verify_enable'):
        runner.add_task(FetchConfigTask())  # NOTE: will keep trying
        runner.add_task(VerifyConfigTask())

    if rospy.has_param('rosbag_enable') and rospy.get_param('rosbag_enable'):
        runner.add_task(RosbagTask())

    # Run the main loop
    runner.run()

