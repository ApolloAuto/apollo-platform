#!/usr/bin/env python
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

# Author: Alexander Sorokin. 
# Based on code from ref_server.cpp by Vijay Pradeep
PKG='actionlib'
import rospy

import sys

from actionlib.simple_action_server import SimpleActionServer
from actionlib.msg import TestAction,TestFeedback


class RefSimpleServer (SimpleActionServer):

    def __init__(self,name):
        action_spec=TestAction
        SimpleActionServer.__init__(self,name,action_spec,self.goal_callback, False);
        self.start()
        rospy.loginfo("Creating SimpleActionServer [%s]\n", name);


    def goal_callback(self,goal):

        rospy.loginfo("Got goal %d", int(goal.goal))
        if goal.goal == 1:
            self.set_succeeded(None, "The ref server has succeeded");
        elif goal.goal == 2:
            self.set_aborted(None, "The ref server has aborted");

        elif goal.goal == 3:
            self.set_aborted(None, "The simple action server can't reject goals");


        elif goal.goal == 4:
            self.set_aborted(None, "Simple server can't save goals");


        elif goal.goal == 5:
            self.set_aborted(None, "Simple server can't save goals");

        elif goal.goal == 6:
            self.set_aborted(None, "Simple server can't save goals");



        elif goal.goal == 7:
            self.set_aborted(None, "Simple server can't save goals");

        elif goal.goal == 8:
            self.set_aborted(None, "Simple server can't save goals");

        elif goal.goal == 9:
            rospy.sleep(1);
            rospy.loginfo("Sending feedback")
            self.publish_feedback(TestFeedback(9)); #by the goal ID
            rospy.sleep(1);
            self.set_succeeded(None, "The ref server has succeeded");


        else:
            pass

if __name__=="__main__":
  rospy.init_node("ref_simple_server");
  ref_server = RefSimpleServer("reference_simple_action");

  rospy.spin();



