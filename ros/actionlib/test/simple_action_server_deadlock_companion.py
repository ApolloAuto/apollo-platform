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
    node = "simple_action_server_deadlock_companion"
    topic = "deadlock"
    max_action_duration = 3

import random

import actionlib
from actionlib.msg import TestAction, TestGoal
from actionlib_msgs.msg import GoalStatus
import rospy


class DeadlockCompanion:

    def __init__(self):
        # Seed random with fully resolved name of node and current time
        random.seed(rospy.get_name() + str(rospy.Time.now().to_sec()))

        # Create actionlib client
        self.action_client = actionlib.SimpleActionClient(
            Constants.topic,
            TestAction)

    def run(self):
        while not rospy.is_shutdown():
            # Send dummy goal
            self.action_client.send_goal(TestGoal())

            # Wait for a random amount of time
            action_duration = random.uniform(0, Constants.max_action_duration)
            self.action_client.wait_for_result(rospy.Duration(action_duration))

            state = self.action_client.get_state()
            if state == GoalStatus.ACTIVE or state == GoalStatus.PENDING:
                self.action_client.cancel_goal()


if __name__ == '__main__':
    rospy.init_node(Constants.node)
    try:
        companion = DeadlockCompanion()
        companion.run()
    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        pass
