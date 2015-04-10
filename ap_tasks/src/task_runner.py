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
                    self._log(event, task, start, False, str(ex.args[0]))
            rospy.loginfo("Task Runner: Completed Event '%s'" % event)

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
        self._folder = "~/bags/"
        self._excl = "\"(.*)(swarm_tracker/swarm_uav_states|network/recv_pose)\""
        self._prefix = "%s%u" % (self._folder, self._acid)
        self._cmd = "rosbag record -a -x " + self._excl + " -o " + self._prefix
        self._proc = None

    def on_status(self):
        subprocess.call("ls %s || mkdir %s" % (self._folder, self._folder),
                        shell=True)
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
        self._folder = "~/blessed/"
        self._id_str = "%03u" % int(self._acid)

    def on_status(self):
        subprocess.call("ls %s || mkdir %s" % (self._folder, self._folder),
                        shell=True)
        for f in ['fence', 'param', 'rally', 'wp']:
            subprocess.call("rm %s%s" % (self._folder, f), shell=True)
        for f in ['fence', 'param', 'rally', 'wp']:
            while True:
                try:
                    cmd = "wget -q -O %s%s http://192.168.2.1/%s/%s" % \
                          (self._folder, f, self._id_str, f)
                    res = subprocess.call(cmd, shell=True)
                    if res == 0: break
                    rospy.logwarn("Error fetching %s; retrying ..." % f)
                except Exception as ex:
                    rospy.logwarn("Exception fetching %s: %s" % \
                                  (f, str(ex.args[0])))
        return True

# Intermediate class for setting lists of things from a file
# Initialization arguments:
#  - name      friendly string name as used by Task
#  - srv_name  name of setter service
#  - srv_type  type of setter service
#  - ext       extension of filename (assumes filename format)
class _SetlistTask(Task):

    def __init__(self, name, srv_name, srv_type, item_attr, ext):
        Task.__init__(self, name)
        self._srv_name = srv_name
        self._srv_type = srv_type
        self._item_attr = item_attr
        self._fname = "%s/blessed/%s" % (os.path.expanduser("~"), ext)

    # Method to parse lines into service req point sub-objects
    # DEFINE IN EACH SUBTYPE
    def _parse(self, *args):
        return None

    # Method to call if/once task succeeds
    # Probably should set a ROS param or publish a message
    def _success(self):
        pass

    def on_status(self):
        self._wait(self._srv_name)
        srv = rospy.ServiceProxy(self._srv_name, self._srv_type)
        with open(self._fname, 'r') as f:
            # Create a request object (NOTE: hackish, relies on service impl)
            req = self._srv_type._request_class()
            # Gets the list attribute of that object
            items = getattr(req, self._item_attr)
            # Add items to request object
            for line in f:
                line = line.rstrip("\n")
                # Ignore comments and blanks
                if not line or line.startswith('#'): continue
                # NOTE: if file is ill-formatted, fail altogether
                try:
                    item = self._parse(*line.split())
                    # Allow parsers to ignore some lines
                    if item is None: continue
                    items.append(item)
                except:
                    raise Exception("Format error: " + str(line))
            # Attempt a few times to set the items
            # NOTE: trust the service call's response to check if it worked
            for i in range(3):
                res = srv(req)
                if not res.ok: continue
                self._success()
                return True
            return False

class ParamTask(_SetlistTask):

    def __init__(self):
        _SetlistTask.__init__(self,
                              "AP Param",
                              "autopilot/fpr_param_setlist",
                              autopilot_srv.ParamSetList,
                              "param",
                              "param")

    def _parse(self, *args):
        p = autopilot_msg.ParamPair()
        p.name = str(args[0]).upper()
        p.value = float(args[1])
        return p

    def _success(self):
        rospy.set_param("ok_param", True)

# Set fence points based on file
class FenceTask(_SetlistTask):

    def __init__(self):
        _SetlistTask.__init__(self,
                              "AP Fence",
                              "autopilot/fpr_fence_setall",
                              autopilot_srv.FenceSetAll,
                              "points",
                              "fence")

    def _parse(self, *args):
        p = autopilot_msg.Fencepoint()
        p.lat = float(args[0])
        p.lon = float(args[1])
        return p

    def _success(self):
        rospy.set_param("ok_fence", True)

# Set rally points based on file
class RallyTask(_SetlistTask):

    def __init__(self):
        _SetlistTask.__init__(self,
                              "AP Rally",
                              "autopilot/fpr_rally_setall",
                              autopilot_srv.RallySetAll,
                              "points",
                              "rally")

    def _parse(self, *args):
        p = autopilot_msg.Rallypoint()
        p.lat = float(args[1]) * 1e7
        p.lon = float(args[2]) * 1e7
        p.alt = float(args[3])
        p.break_alt = float(args[4])
        p.land_dir = float(args[5]) * 1e2
        p.flags = int(args[6])
        return p

    def _success(self):
        rospy.set_param("ok_rally", True)

# Set waypoints based on file
# NOTE: Assumes a "v1.10" formatted waypoint file
class WPTask(_SetlistTask):

    def __init__(self):
        _SetlistTask.__init__(self,
                              "AP WP",
                              "autopilot/wp_setall",
                              autopilot_srv.WPSetAll,
                              "points",
                              "wp")

    def _parse(self, *args):
        if args[0] == 'QGC':
            if args[2] != '110':
                raise Exception("")
            return None
        p = autopilot_msg.Waypoint()
        p.seq = int(args[0])
        p.frame = int(args[2])
        p.command = int(args[3])
        p.current = bool(int(args[1]))
        p.autocontinue = bool(int(args[11]))
        p.param1 = float(args[4])
        p.param2 = float(args[5])
        p.param3 = float(args[6])
        p.param4 = float(args[7])
        p.x = float(args[8])
        p.y = float(args[9])
        p.z = float(args[10])
        return p

    def _success(self):
        rospy.set_param("ok_wp", True)

#-----------------------------------------------------------------------
# Start-up

if __name__ == '__main__':
    # ROS initialization
    rospy.init_node('task_runner')
    
    # Instantiate TaskRunner and Task instances
    runner = TaskRunner()
    runner.add_task(SetIDTask())
    runner.add_task(TxPowerTask())

    if rospy.has_param('verify_enable') and rospy.get_param('verify_enable'):
        runner.add_task(FetchConfigTask())  # NOTE: will keep trying
        runner.add_task(RallyTask())
        runner.add_task(WPTask())
        runner.add_task(FenceTask())
        runner.add_task(ParamTask())

    if rospy.has_param('rosbag_enable') and rospy.get_param('rosbag_enable'):
        runner.add_task(RosbagTask())

    # Run the main loop
    runner.run()

