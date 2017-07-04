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
PKG='actionlib'

import sys
import unittest
import rospy
from actionlib_msgs.msg import *
from actionlib import SimpleActionClient
from actionlib import ActionClient
from actionlib.msg import TestAction, TestGoal

class TestRefSimpleActionServer(unittest.TestCase):

    def test_one(self):
        client = SimpleActionClient('reference_simple_action', TestAction)
        self.assert_(client.wait_for_server(rospy.Duration(2.0)),
                     'Could not connect to the action server')

        goal = TestGoal(1)
        client.send_goal(goal)
        self.assert_(client.wait_for_result(rospy.Duration(2.0)),
                     "Goal didn't finish")
        self.assertEqual(GoalStatus.SUCCEEDED, client.get_state())

        goal = TestGoal(2)
        client.send_goal(goal)
        self.assert_(client.wait_for_result(rospy.Duration(10.0)),
                     "Goal didn't finish")
        self.assertEqual(GoalStatus.ABORTED, client.get_state())

        goal = TestGoal(3)
        client.send_goal(goal)
        self.assert_(client.wait_for_result(rospy.Duration(10.0)),
                     "Goal didn't finish")

        #The simple server can't reject goals
        self.assertEqual(GoalStatus.ABORTED, client.get_state())


        goal = TestGoal(9)
        saved_feedback={};
        def on_feedback(fb):
            rospy.loginfo("Got feedback")
            saved_feedback[0]=fb

        client.send_goal(goal,feedback_cb=on_feedback)
        self.assert_(client.wait_for_result(rospy.Duration(10.0)),
                     "Goal didn't finish")
        self.assertEqual(GoalStatus.SUCCEEDED, client.get_state())

        self.assertEqual(saved_feedback[0].feedback,9)




if __name__ == '__main__':
    import rostest
    rospy.init_node('test_ref_simple_action_server')
    rostest.rosrun('actionlib', 'test_simple_action_client_python', TestRefSimpleActionServer)
