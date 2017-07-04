#! /usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
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
#     * Neither the name of the Willow Garage, Inc. nor the names of its
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

PKG = 'actionlib'

import sys
import unittest
import rospy
import rostest
from actionlib import SimpleActionClient
from actionlib_msgs.msg import *
from actionlib.msg import *


class SimpleExerciser(unittest.TestCase):

    def setUp(self):
        self.default_wait = rospy.Duration(60.0)
        self.client = SimpleActionClient('test_request_action', TestRequestAction)
        self.assert_(self.client.wait_for_server(self.default_wait))

    def test_just_succeed(self):
        goal = TestRequestGoal(terminate_status = TestRequestGoal.TERMINATE_SUCCESS,
                               the_result = 42)
        self.client.send_goal(goal)
        self.client.wait_for_result(self.default_wait)

        self.assertEqual(GoalStatus.SUCCEEDED, self.client.get_state())
        self.assertEqual(42, self.client.get_result().the_result)

    def test_just_abort(self):
        goal = TestRequestGoal(terminate_status = TestRequestGoal.TERMINATE_ABORTED,
                               the_result = 42)
        self.client.send_goal(goal)
        self.client.wait_for_result(self.default_wait)

        self.assertEqual(GoalStatus.ABORTED, self.client.get_state())
        self.assertEqual(42, self.client.get_result().the_result)

    def test_just_preempt(self):
        goal = TestRequestGoal(terminate_status = TestRequestGoal.TERMINATE_SUCCESS,
                               delay_terminate = rospy.Duration(100000),
                               the_result = 42)
        self.client.send_goal(goal)

        # Ensure that the action server got the goal, before continuing
        timeout_time = rospy.Time.now() + self.default_wait
        while rospy.Time.now() < timeout_time:
            if (self.client.get_state() != GoalStatus.PENDING):
                break
        self.client.cancel_goal()

        self.client.wait_for_result(self.default_wait)
        self.assertEqual(GoalStatus.PREEMPTED, self.client.get_state())
        self.assertEqual(42, self.client.get_result().the_result)

    # Should print out errors about not setting a terminal status in the action server.
    def test_drop(self):
        goal = TestRequestGoal(terminate_status = TestRequestGoal.TERMINATE_DROP,
                               the_result = 42)
        self.client.send_goal(goal)
        self.client.wait_for_result(self.default_wait)

        self.assertEqual(GoalStatus.ABORTED, self.client.get_state())
        self.assertEqual(0, self.client.get_result().the_result)

    # Should print out errors about throwing an exception
    def test_exception(self):
        goal = TestRequestGoal(terminate_status = TestRequestGoal.TERMINATE_EXCEPTION,
                               the_result = 42)
        self.client.send_goal(goal)
        self.client.wait_for_result(self.default_wait)

        self.assertEqual(GoalStatus.ABORTED, self.client.get_state())
        self.assertEqual(0, self.client.get_result().the_result)

    def test_ignore_cancel_and_succeed(self):
        goal = TestRequestGoal(terminate_status = TestRequestGoal.TERMINATE_SUCCESS,
                               delay_terminate = rospy.Duration(2.0),
                               ignore_cancel = True,
                               the_result = 42)
        self.client.send_goal(goal)

        # Ensure that the action server got the goal, before continuing
        timeout_time = rospy.Time.now() + self.default_wait
        while rospy.Time.now() < timeout_time:
            if (self.client.get_state() != GoalStatus.PENDING):
                break
        self.client.cancel_goal()

        self.client.wait_for_result(self.default_wait)

        self.assertEqual(GoalStatus.SUCCEEDED, self.client.get_state())
        self.assertEqual(42, self.client.get_result().the_result)


    def test_lose(self):
        goal = TestRequestGoal(terminate_status = TestRequestGoal.TERMINATE_LOSE,
                               the_result = 42)
        self.client.send_goal(goal)
        self.client.wait_for_result(self.default_wait)

        self.assertEqual(GoalStatus.LOST, self.client.get_state())

    # test_freeze_server has been removed, as it is undecided what should happen
    # when the action server disappears.
'''
    def test_freeze_server(self):
        goal = TestRequestGoal(terminate_status = TestRequestGoal.TERMINATE_SUCCESS,
                               the_result = 42,
                               pause_status = rospy.Duration(10.0))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(13.0))

        self.assertEqual(GoalStatus.LOST, self.client.get_state())
'''



if __name__ == '__main__':
    rospy.init_node("exercise_simple_server")
    rostest.run(PKG, 'exercise_simple_server', SimpleExerciser, sys.argv)
