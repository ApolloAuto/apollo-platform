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

    def testsimple(self):
        return
        client = ActionClient('reference_action', TestAction)
        self.assert_(client.wait_for_action_server_to_start(rospy.Duration(2.0)),
                     'Could not connect to the action server')

        goal = TestGoal(1)
        client.send_goal(goal)
        self.assert_(client.wait_for_goal_to_finish(rospy.Duration(2.0)),
                     "Goal didn't finish")
        self.assertEqual(GoalStatus.SUCCEEDED, client.get_terminal_state())
        self.assertEqual(GoalStatus.SUCCEEDED, client.get_state())

        goal = TestGoal(2)
        client.send_goal(goal)
        self.assert_(client.wait_for_goal_to_finish(rospy.Duration(10.0)),
                     "Goal didn't finish")
        self.assertEqual(GoalStatus.ABORTED, client.get_terminal_state())
        self.assertEqual(GoalStatus.ABORTED, client.get_state())


    def test_abort(self):
        client = ActionClient('reference_action', TestAction)
        self.assert_(client.wait_for_server(rospy.Duration(2.0)),
                     'Could not connect to the action server')

        goal_work = TestGoal(4)
        goal_abort = TestGoal(6)
        goal_feedback = TestGoal(8)

        g1=client.send_goal(goal_work)
        g2=client.send_goal(goal_work)
        g3=client.send_goal(goal_work)
        g4=client.send_goal(goal_work)

        rospy.sleep(0.5);
        self.assertEqual(g1.get_goal_status(),GoalStatus.ACTIVE) #,"Should be active")
        self.assertEqual(g2.get_goal_status(),GoalStatus.ACTIVE,"Should be active")
        self.assertEqual(g3.get_goal_status(),GoalStatus.ACTIVE,"Shoule be active")
        self.assertEqual(g4.get_goal_status(),GoalStatus.ACTIVE,"Should be active")

        g5=client.send_goal(goal_abort)
        rospy.sleep(0.5);
        self.assertEqual(g5.get_goal_status(),GoalStatus.SUCCEEDED,"Should be done")

        self.assertEqual(g1.get_goal_status(),GoalStatus.ABORTED,"Should be aborted")
        self.assertEqual(g2.get_goal_status(),GoalStatus.ABORTED,"Should be aborted")
        self.assertEqual(g3.get_goal_status(),GoalStatus.ABORTED,"Shoule be aborted")
        self.assertEqual(g4.get_goal_status(),GoalStatus.ABORTED,"Should be aborted")



    def test_feedback(self):
        client = ActionClient('reference_action', TestAction)
        self.assert_(client.wait_for_server(rospy.Duration(2.0)),
                     'Could not connect to the action server')

        goal_work = TestGoal(4)
        goal_abort = TestGoal(6)
        goal_feedback = TestGoal(7)

        rospy.logwarn("This is a hacky way to associate goals with feedback");
        feedback={};
        def update_feedback(id,g,f):
            feedback[id]=f;

        g1=client.send_goal(goal_work,feedback_cb=lambda g,f:update_feedback(0,g,f))
        g2=client.send_goal(goal_work,feedback_cb=lambda g,f:update_feedback(1,g,f))
        g3=client.send_goal(goal_work,feedback_cb=lambda g,f:update_feedback(2,g,f))
        g4=client.send_goal(goal_work,feedback_cb=lambda g,f:update_feedback(3,g,f))

        rospy.sleep(0.5);
        self.assertEqual(g1.get_goal_status(),GoalStatus.ACTIVE,"Should be active")
        self.assertEqual(g2.get_goal_status(),GoalStatus.ACTIVE,"Should be active")
        self.assertEqual(g3.get_goal_status(),GoalStatus.ACTIVE,"Shoule be active")
        self.assertEqual(g4.get_goal_status(),GoalStatus.ACTIVE,"Should be active")

        g5=client.send_goal(goal_feedback)
        rospy.sleep(0.5);
        self.assertEqual(g5.get_goal_status(),GoalStatus.SUCCEEDED,"Should be done")


        self.assertEqual(g1.get_goal_status(),GoalStatus.ACTIVE)
        self.assertEqual(feedback[0].feedback,4)
        self.assertEqual(g2.get_goal_status(),GoalStatus.ACTIVE)
        self.assertEqual(feedback[1].feedback,3)
        self.assertEqual(g3.get_goal_status(),GoalStatus.ACTIVE)
        self.assertEqual(feedback[2].feedback,2)
        self.assertEqual(g4.get_goal_status(),GoalStatus.ACTIVE)
        self.assertEqual(feedback[3].feedback,1)

        g6=client.send_goal(goal_abort)
        rospy.sleep(0.5);


    def test_result(self):
        client = ActionClient('reference_action', TestAction)
        self.assert_(client.wait_for_server(rospy.Duration(2.0)),
                     'Could not connect to the action server')

        goal_work = TestGoal(4)
        goal_abort = TestGoal(6)
        goal_result = TestGoal(8)

        rospy.logwarn("This is a hacky way to associate goals with feedback");

        g1=client.send_goal(goal_work)
        g2=client.send_goal(goal_work)
        g3=client.send_goal(goal_work)
        g4=client.send_goal(goal_work)

        rospy.sleep(0.5);
        self.assertEqual(g1.get_goal_status(),GoalStatus.ACTIVE,"Should be active")
        self.assertEqual(g2.get_goal_status(),GoalStatus.ACTIVE,"Should be active")
        self.assertEqual(g3.get_goal_status(),GoalStatus.ACTIVE,"Shoule be active")
        self.assertEqual(g4.get_goal_status(),GoalStatus.ACTIVE,"Should be active")

        g5=client.send_goal(goal_result)
        rospy.sleep(0.5);
        self.assertEqual(g5.get_goal_status(),GoalStatus.SUCCEEDED,"Should be done")

        self.assertEqual(g1.get_goal_status(),GoalStatus.SUCCEEDED)
        self.assertEqual(g1.get_result().result,4)
        self.assertEqual(g2.get_goal_status(),GoalStatus.ABORTED)
        self.assertEqual(g2.get_result().result,3)
        self.assertEqual(g3.get_goal_status(),GoalStatus.SUCCEEDED)
        self.assertEqual(g3.get_result().result,2)
        self.assertEqual(g4.get_goal_status(),GoalStatus.ABORTED)
        self.assertEqual(g4.get_result().result,1)
        g6=client.send_goal(goal_abort)
        rospy.sleep(0.5);





if __name__ == '__main__':
    import rostest
    rospy.init_node('test_ref_simple_action_server')
    rostest.rosrun('actionlib', 'test_simple_action_client_python', TestRefSimpleActionServer)
