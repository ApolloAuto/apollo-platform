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

from actionlib.action_server import ActionServer
from actionlib.msg import TestAction,TestFeedback,TestResult

class RefServer (ActionServer):

    def __init__(self,name):
        action_spec=TestAction
        ActionServer.__init__(self,name,action_spec,self.goalCallback,self.cancelCallback, False);
        self.start()
        rospy.loginfo("Creating ActionServer [%s]\n", name);

        self.saved_goals=[]

    def goalCallback(self,gh):
        goal = gh.get_goal();

        rospy.loginfo("Got goal %d", int(goal.goal))
        if goal.goal == 1:
            gh.set_accepted();
            gh.set_succeeded(None, "The ref server has succeeded");
        elif goal.goal == 2:
            gh.set_accepted();
            gh.set_aborted(None, "The ref server has aborted");
        elif goal.goal == 3:
            gh.set_rejected(None, "The ref server has rejected");


        elif goal.goal == 4:
            
            self.saved_goals.append(gh);
            gh.set_accepted();

        elif goal.goal == 5:

            gh.set_accepted();
            for g in self.saved_goals:
                g.set_succeeded();
            self.saved_goals = [];
            gh.set_succeeded();


        elif goal.goal == 6:
            gh.set_accepted();
            for g in self.saved_goals:
                g.set_aborted();
            self.saved_goals = [];
            gh.set_succeeded();

        elif goal.goal == 7:
            gh.set_accepted();
            n=len(self.saved_goals);
            for i,g in enumerate(self.saved_goals):
                g.publish_feedback(TestFeedback(n-i));

            gh.set_succeeded();

        elif goal.goal == 8:
            gh.set_accepted();
            n=len(self.saved_goals);
            for i,g in enumerate(self.saved_goals):
                if i % 2 ==0:
                    g.set_succeeded(TestResult(n-i), "The ref server has succeeded");
                else:
                    g.set_aborted(TestResult(n-i), "The ref server has aborted")
            self.saved_goals=[];
            gh.set_succeeded();


        else:
            pass

    def cancelCallback(self,gh):
        pass

if __name__=="__main__":
  rospy.init_node("ref_server");
  ref_server = RefServer("reference_action");

  rospy.spin();



