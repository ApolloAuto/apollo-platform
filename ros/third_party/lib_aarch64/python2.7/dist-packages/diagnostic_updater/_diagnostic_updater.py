# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-

""" diagnostic_updater for Python.
@author Brice Rebsamen <brice [dot] rebsamen [gmail]>
"""

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import threading, httplib
from diagnostic_msgs.msg import DiagnosticArray

from ._diagnostic_status_wrapper import *

class DiagnosticTask:
    """DiagnosticTask is an abstract base class for collecting diagnostic data.

    Subclasses are provided for generating common diagnostic information.
    A DiagnosticTask has a name, and a function that is called to cleate a
    DiagnosticStatusWrapper.
    """

    def __init__(self, name):
        """Constructs a DiagnosticTask setting its name in the process."""
        self.name = name

    def getName(self):
        """Returns the name of the DiagnosticTask."""
        return self.name

    def run(self, stat):
        """Fills out this Task's DiagnosticStatusWrapper.
        @param stat: the DiagnosticStatusWrapper to fill
        @return the filled DiagnosticStatusWrapper
        """
        return stat



class FunctionDiagnosticTask(DiagnosticTask):
    """A DiagnosticTask based on a function.

    The FunctionDiagnosticTask calls the function when it updates. The
    function updates the DiagnosticStatusWrapper and collects data.

    This is useful for gathering information about a device or driver, like temperature,
    calibration, etc.
    """

    def __init__(self, name, fn):
        """Constructs a GenericFunctionDiagnosticTask based on the given name and function.
        @param name Name of the function.
        @param fn Function to be called when run is called.
        """
        DiagnosticTask.__init__(self, name)
        self.fn = fn

    def run(self, stat):
        return self.fn(stat)



class CompositeDiagnosticTask(DiagnosticTask):
    """Merges CompositeDiagnosticTask into a single DiagnosticTask.

    The CompositeDiagnosticTask allows multiple DiagnosticTask instances to
    be combined into a single task that produces a single single
    DiagnosticStatusWrapped. The output of the combination has the max of
    the status levels, and a concatenation of the non-zero-level messages.

    For instance, this could be used to combine the calibration and offset data
    from an IMU driver.
    """

    def __init__(self, name):
        """Constructs a CompositeDiagnosticTask with the given name."""
        DiagnosticTask.__init__(self, name)
        self.tasks = []

    def run(self, stat):
        """Runs each child and merges their outputs."""
        combined_summary = DiagnosticStatusWrapper()
        original_summary = DiagnosticStatusWrapper()

        original_summary.summary(stat)

        for task in self.tasks:
            # Put the summary that was passed in.
            stat.summary(original_summary)
            # Let the next task add entries and put its summary.
            stat = task.run(stat)
            # Merge the new summary into the combined summary.
            combined_summary.mergeSummary(stat)


        # Copy the combined summary into the output.
        stat.summary(combined_summary)
        return stat

    def addTask(self, t):
        """Adds a child CompositeDiagnosticTask.

        This CompositeDiagnosticTask will be called each time this
        CompositeDiagnosticTask is run.
        """
        self.tasks.append(t)



class DiagnosticTaskVector:
    """Internal use only.

    Base class for diagnostic_updater::Updater and self_test::Dispatcher.
    The class manages a collection of diagnostic updaters. It contains the
    common functionality used for producing diagnostic updates and for
    self-tests.
    """

    class DiagnosticTaskInternal:
        """Class used to represent a diagnostic task internally in
        DiagnosticTaskVector.
        """

        def __init__(self, name, fn):
            self.name = name
            self.fn = fn

        def run(self, stat):
            stat.name = self.name
            return self.fn(stat)


    def __init__(self):
        self.tasks = []
        self.lock = threading.Lock()

    def addedTaskCallback(self, task):
        """Allows an action to be taken when a task is added. The Updater class
        uses this to immediately publish a diagnostic that says that the node
        is loading.
        """
        pass

    def add(self, *args):
        """Add a task to the DiagnosticTaskVector.

        Usage:
        add(task): where task is a DiagnosticTask
        add(name, fn): add a DiagnosticTask embodied by a name and function
        """
        if len(args)==1:
            task = DiagnosticTaskVector.DiagnosticTaskInternal(args[0].getName(), args[0].run)
        elif len(args)==2:
            task = DiagnosticTaskVector.DiagnosticTaskInternal(args[0], args[1])

        with self.lock:
            self.tasks.append(task)
            self.addedTaskCallback(task)

    def removeByName(self, name):
        """Removes a task based on its name.

        Removes the first task that matches the specified name. (New in
        version 1.1.2)

        @param name Name of the task to remove.
        @return Returns true if a task matched and was removed.
        """
        found = False
        with self.lock:
            for i in range(len(self.tasks)):
                if self.tasks[i].name == name:
                    self.tasks.pop(i)
                    found = True
                    break
        return found




class Updater(DiagnosticTaskVector):
    """Manages a list of diagnostic tasks, and calls them in a rate-limited manner.

    This class manages a list of diagnostic tasks. Its update function
    should be called frequently. At some predetermined rate, the update
    function will cause all the diagnostic tasks to run, and will collate
    and publish the resulting diagnostics. The publication rate is
    determined by the "~diagnostic_period" ros parameter.

    The class also allows an update to be forced when something significant
    has happened, and allows a single message to be broadcast on all the
    diagnostics if normal operation of the node is suspended for some
    reason.
    """

    def __init__(self):
        """Constructs an updater class."""
        DiagnosticTaskVector.__init__(self)
        self.publisher = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)

        self.last_time = rospy.Time.now()
        self.last_time_period_checked = self.last_time
        self.period = 1

        self.verbose = False
        self.hwid = ""
        self.warn_nohwid_done = False

    def update(self):
        """Causes the diagnostics to update if the inter-update interval
        has been exceeded.
        """
        self._check_diagnostic_period()
        if rospy.Time.now() >= self.last_time + rospy.Duration(self.period):
            self.force_update()

    def force_update(self):
        """Forces the diagnostics to update.

        Useful if the node has undergone a drastic state change that should be
        published immediately.
        """
        self.last_time = rospy.Time.now()

        warn_nohwid = len(self.hwid)==0

        status_vec = []

        with self.lock: # Make sure no adds happen while we are processing here.
            for task in self.tasks:
                status = DiagnosticStatusWrapper()
                status.name = task.name
                status.level = 2
                status.message = "No message was set"
                status.hardware_id = self.hwid

                stat = task.run(status)

                status_vec.append(status)

                if status.level:
                    warn_nohwid = False

                if self.verbose and status.level:
                    rospy.logwarn("Non-zero diagnostic status. Name: '%s', status %i: '%s'" %
                                (status.name, status.level, status.message))

        if warn_nohwid and not self.warn_nohwid_done:
            rospy.logwarn("diagnostic_updater: No HW_ID was set. This is probably a bug. Please report it. For devices that do not have a HW_ID, set this value to 'none'. This warning only occurs once all diagnostics are OK so it is okay to wait until the device is open before calling setHardwareID.");
            self.warn_nohwid_done = True

        self.publish(status_vec)

    def broadcast(self, lvl, msg):
        """Outputs a message on all the known DiagnosticStatus.

        Useful if something drastic is happening such as shutdown or a self-test.

        @param lvl Level of the diagnostic being output.
        @param msg Status message to output.
        """

        status_vec = []

        for task in self.tasks:
            status = DiagnosticStatusWrapper()
            status.name = task.name
            status.summary(lvl, msg)
            status_vec.append(status)

        self.publish(status_vec)

    def setHardwareID(self, hwid):
        self.hwid = hwid

    def _check_diagnostic_period(self):
        """Recheck the diagnostic_period on the parameter server."""

        # This was getParamCached() call in the cpp code. i.e. it would throttle
        # the actual call to the parameter server using a notification of change
        # mechanism.
        # This is not available in rospy. Hence I throttle the call to the
        # parameter server using a standard timeout mechanism (4Hz)

        now = rospy.Time.now()
        if  now >= self.last_time_period_checked + rospy.Duration(0.25):
            try:
                self.period = rospy.get_param("~diagnostic_period", 1)
                self.last_time_period_checked = now
            except (httplib.CannotSendRequest, httplib.ResponseNotReady):
                pass

    def publish(self, msg):
        """Publishes a single diagnostic status or a vector of diagnostic statuses."""
        if not type(msg) is list:
            msg = [msg]

        for stat in msg:
            stat.name = rospy.get_name()[1:]+ ": " + stat.name

        da = DiagnosticArray()
        da.status = msg
        da.header.stamp = rospy.Time.now() # Add timestamp for ROS 0.10
        self.publisher.publish(da)

    def addedTaskCallback(self, task):
        stat = DiagnosticStatusWrapper()
        stat.name = task.name
        stat.summary(0, "Node starting up")
        self.publish(stat)
