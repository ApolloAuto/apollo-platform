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

import sys
import time
import rospy
import rostest

from actionlib.simple_action_server import SimpleActionServer
from actionlib.server_goal_handle import ServerGoalHandle;
from actionlib.msg import *

class RefSimpleServer(SimpleActionServer):

    def __init__(self, name):
        SimpleActionServer.__init__(self, name, TestRequestAction, self.execute_cb, False)
        self.start()

    def execute_cb(self, goal):
        rospy.logdebug("Goal:\n" + str(goal))
        result = TestRequestResult(goal.the_result, True)

        if goal.pause_status > rospy.Duration(0.0):
            rospy.loginfo("Locking the action server for %.3f seconds" % goal.pause_status.to_sec())
            status_continue_time = rospy.get_rostime() + goal.pause_status
            # Takes the action server lock to prevent status from
            # being published (simulates a freeze).
            with self.action_server.lock:
                while rospy.get_rostime() < status_continue_time:
                    time.sleep(0.02)
                rospy.loginfo("Unlocking the action server")


        terminate_time = rospy.get_rostime() + goal.delay_terminate
        while rospy.get_rostime() < terminate_time:
            time.sleep(0.02)
            if not goal.ignore_cancel:
                if self.is_preempt_requested():
                    self.set_preempted(result, goal.result_text)
                    return

        rospy.logdebug("Terminating goal as: %i" % goal.terminate_status)

        if goal.terminate_status == TestRequestGoal.TERMINATE_SUCCESS:
            self.set_succeeded(result, goal.result_text)
        elif goal.terminate_status == TestRequestGoal.TERMINATE_ABORTED:
            self.set_aborted(result, goal.result_text)
        elif goal.terminate_status == TestRequestGoal.TERMINATE_REJECTED:
            rospy.logerr("Simple action server cannot reject goals")
            self.set_aborted(None, "Simple action server cannot reject goals")
        elif goal.terminate_status == TestRequestGoal.TERMINATE_DROP:
            rospy.loginfo("About to drop the goal.  This should produce an error message.")
            return
        elif goal.terminate_status == TestRequestGoal.TERMINATE_EXCEPTION:
            rospy.loginfo("About to throw an exception.  This should produce an error message.")
            raise Exception("Terminating by throwing an exception")
        elif goal.terminate_status == TestRequestGoal.TERMINATE_LOSE:
            # Losing the goal requires messing about in the action server's innards
            for i, s in enumerate(self.action_server.status_list):
                if s.status.goal_id.id == self.current_goal.goal.goal_id.id:
                    del self.action_server.status_list[i]
                    break
            self.current_goal = ServerGoalHandle()
        else:
            rospy.logerr("Don't know how to terminate as %d" % goal.terminate_status)
            self.set_aborted(None, "Don't know how to terminate as %d" % goal.terminate_status)



if __name__ == '__main__':
    rospy.init_node("ref_simple_server")
    ref_server = RefSimpleServer("test_request_action")
    print "Spinning"
    rospy.spin()
