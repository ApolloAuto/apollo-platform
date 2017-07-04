#! /usr/bin/env python
#
# Copyright (c) 2013, Miguel Sarabia
# Imperial College London
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Imperial College London nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#


class Constants:
    pkg = "actionlib"
    node = "test_simple_action_server_deadlock"
    topic = "deadlock"
    deadlock_timeout = 45  # in seconds
    shutdown_timeout = 2  # in seconds
    max_action_duration = 3

import random
import sys
import threading
import unittest

import actionlib
from actionlib.msg import TestAction
import rosnode
import rospy


class DeadlockTest(unittest.TestCase):

    def test_deadlock(self):
        # Prepare condition (for safe preemption)
        self.condition = threading.Condition()
        self.last_execution_time = None

        # Prepare Simple Action Server
        self.action_server = actionlib.SimpleActionServer(
            Constants.topic,
            TestAction,
            execute_cb=self.execute_callback,
            auto_start=False)

        self.action_server.register_preempt_callback(self.preempt_callback)
        self.action_server.start()

        # Sleep for the amount specified
        rospy.sleep(Constants.deadlock_timeout)

        # Start actual tests
        running_nodes = set(rosnode.get_node_names())
        required_nodes = {
            "/deadlock_companion_1",
            "/deadlock_companion_2",
            "/deadlock_companion_3",
            "/deadlock_companion_4",
            "/deadlock_companion_5"}

        self.assertTrue(required_nodes.issubset(running_nodes),
            "Required companion nodes are not currently running")

        # Shutdown companions so that we can exit nicely
        termination_time = rospy.Time.now()
        rosnode.kill_nodes(required_nodes)

        rospy.sleep(Constants.shutdown_timeout)

        # Check last execution wasn't too long ago...
        self.assertIsNotNone(self.last_execution_time is None,
            "Execute Callback was never executed")

        time_since_last_execution = (
            termination_time - self.last_execution_time).to_sec()

        self.assertTrue(
            time_since_last_execution < 2 * Constants.max_action_duration,
            "Too long since last goal was executed; likely due to a deadlock")

    def execute_callback(self, goal):
        # Note down last_execution time
        self.last_execution_time = rospy.Time.now()

        # Determine duration of this action
        action_duration = random.uniform(0, Constants.max_action_duration)

        with self.condition:
            if not self.action_server.is_preempt_requested():
                self.condition.wait(action_duration)

        if self.action_server.is_preempt_requested():
            self.action_server.set_preempted()
        else:
            self.action_server.set_succeeded()

    def preempt_callback(self):
        with self.condition:
            self.condition.notify()


if __name__ == '__main__':
    import rostest
    rospy.init_node(Constants.node)
    rostest.rosrun(Constants.pkg, Constants.node, DeadlockTest)
