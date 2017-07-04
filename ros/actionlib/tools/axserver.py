#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

"""
usage: %prog /action_name action_type
"""

PKG='actionlib'

from optparse import OptionParser
import roslib.message
import wx
import sys
import rospy
import actionlib
import time
import threading
from cStringIO import StringIO
from library import *
from dynamic_action import DynamicAction

SEND_FEEDBACK = 0
SUCCEED = 1
ABORT = 2
PREEMPT = 3

class AXServerApp(wx.App):
    def __init__(self, action_type, action_name):
        self.action_type = action_type
        wx.App.__init__(self)

        self.server = actionlib.SimpleActionServer(action_name, self.action_type.action, self.execute)
        self.condition = threading.Condition()
        self.feedback_msg = None
        self.result_msg = None
        self.execute_type = None


    def set_goal(self, goal):
        if goal is None:
            self.status_bg.SetBackgroundColour(wx.Colour(200,0,0))
            self.status.SetLabel("Waiting For Goal...")
            self.send_feedback.Disable()
            self.succeed.Disable()
            self.abort.Disable()
            self.preempt.Disable()

            self.goal.SetValue("")

        else:
            self.status_bg.SetBackgroundColour(wx.Colour(0,200,0))
            self.status.SetLabel("Received Goal.  Send feedback, succeed, or abort.")
            self.send_feedback.Enable()
            self.succeed.Enable()
            self.abort.Enable()
            self.preempt.Enable()

            try:
                self.goal.SetValue(to_yaml(goal))
            except UnicodeDecodeError:
                self.goal.SetValue("Cannot display goal due to unprintable characters")

    def set_preempt_requested(self):
        self.status_bg.SetBackgroundColour(wx.Colour(0, 200, 200))
        self.status.SetLabel("Preempt requested...")

    def execute(self, goal):

        wx.CallAfter(self.set_goal, goal)
        self.condition.acquire()

        self.result_msg = None
        self.feedback_msg = None
        self.execute_type = None

        while self.execute_type is None or self.execute_type == SEND_FEEDBACK:
            self.result_msg = None
            self.feedback_msg = None
            self.execute_type = None

            while self.execute_type is None:
                if self.server.is_preempt_requested():
                    wx.CallAfter(self.set_preempt_requested)
                self.condition.wait(1.0)

            if self.execute_type == SEND_FEEDBACK:
                if self.feedback_msg is not None:
                    self.server.publish_feedback(self.feedback_msg)


        if self.execute_type == SUCCEED:
            self.server.set_succeeded(self.result_msg)

        if self.execute_type == ABORT:
            self.server.set_aborted()

        if self.execute_type == PREEMPT:
            self.server.set_preempted()

        wx.CallAfter(self.set_goal, None)

        self.condition.release()

    def on_feedback(self, event):
        self.condition.acquire()

        try:
            self.feedback_msg = yaml_msg_str(self.action_type.feedback,
                                             self.feedback.GetValue())
            buff = StringIO()
            self.feedback_msg.serialize(buff)

            self.execute_type = SEND_FEEDBACK
            self.condition.notify()
        except roslib.message.SerializationError, e:
            self.feedback_msg = None
            wx.MessageBox(str(e), "Error serializing feedback", wx.OK)

        self.condition.release()



    def on_succeed(self, event):
        self.condition.acquire()

        try:
            self.result_msg = yaml_msg_str(self.action_type.result, self.result.GetValue())
            buff = StringIO()
            self.result_msg.serialize(buff)

            self.execute_type = SUCCEED
            self.condition.notify()
        except roslib.message.SerializationError, e:
            self.result_msg = None
            wx.MessageBox(str(e), "Error serializing result", wx.OK)

        self.condition.release()

    def on_abort(self, event):
        self.condition.acquire()

        self.execute_type = ABORT
        self.condition.notify()

        self.condition.release()

    def on_preempt(self, event):
        self.condition.acquire()

        self.execute_type = PREEMPT
        self.condition.notify()

        self.condition.release()

    def OnInit(self):

        self.frame = wx.Frame(None, -1, self.action_type.name + ' Standin')

        self.sz = wx.BoxSizer(wx.VERTICAL)

        tmp_feedback = self.action_type.feedback()
        tmp_result = self.action_type.result()

        self.goal = wx.TextCtrl(self.frame, -1, style=(wx.TE_MULTILINE | wx.TE_READONLY))
        self.goal_st_bx = wx.StaticBox(self.frame, -1, "Goal")
        self.goal_st = wx.StaticBoxSizer(self.goal_st_bx, wx.VERTICAL)
        self.goal_st.Add(self.goal, 1, wx.EXPAND)

        self.feedback = wx.TextCtrl(self.frame, -1, style=wx.TE_MULTILINE)
        self.feedback.SetValue(to_yaml(tmp_feedback))
        self.feedback_st_bx = wx.StaticBox(self.frame, -1, "Feedback")
        self.feedback_st = wx.StaticBoxSizer(self.feedback_st_bx, wx.VERTICAL)
        self.feedback_st.Add(self.feedback, 1, wx.EXPAND)

        self.result = wx.TextCtrl(self.frame, -1, style=wx.TE_MULTILINE)
        self.result.SetValue(to_yaml(tmp_result))
        self.result_st_bx = wx.StaticBox(self.frame, -1, "Result")
        self.result_st = wx.StaticBoxSizer(self.result_st_bx, wx.VERTICAL)
        self.result_st.Add(self.result, 1, wx.EXPAND)

        self.send_feedback = wx.Button(self.frame, -1, label="SEND FEEDBACK")
        self.send_feedback.Bind(wx.EVT_BUTTON, self.on_feedback)

        self.succeed = wx.Button(self.frame, -1, label="SUCCEED")
        self.succeed.Bind(wx.EVT_BUTTON, self.on_succeed)

        self.abort    = wx.Button(self.frame, -1, label="ABORT")
        self.abort.Bind(wx.EVT_BUTTON, self.on_abort)

        self.preempt    = wx.Button(self.frame, -1, label="PREEMPT")
        self.preempt.Bind(wx.EVT_BUTTON, self.on_preempt)

        self.status_bg = wx.Panel(self.frame, -1)
        self.status_bg.SetBackgroundColour(wx.Colour(200,0,0))
        self.status = wx.StaticText(self.status_bg, -1, label="Waiting For Goal...")

        self.sz.Add(self.goal_st, 1, wx.EXPAND)
        self.sz.Add(self.feedback_st, 1, wx.EXPAND)
        self.sz.Add(self.result_st, 1, wx.EXPAND)
        self.sz.Add(self.send_feedback, 0, wx.EXPAND)
        self.sz.Add(self.succeed, 0, wx.EXPAND)
        self.sz.Add(self.abort, 0, wx.EXPAND)
        self.sz.Add(self.preempt, 0, wx.EXPAND)
        self.sz.Add(self.status_bg, 0, wx.EXPAND)

        self.frame.SetSizer(self.sz)

        self.set_goal(None)

        self.sz.Layout()
        self.frame.Show()

        return True

if __name__ == '__main__':
    rospy.init_node('axserver', anonymous=True)

    parser = OptionParser(__doc__.strip())
#    parser.add_option("-t","--test",action="store_true", dest="test",default=False,
#                      help="A testing flag")
#  parser.add_option("-v","--var",action="store",type="string", dest="var",default="blah")

    (options, args) = parser.parse_args(rospy.myargv())

    if (len(args) != 3):
        parser.error("You must specify the action name and type. Eg: ./axserver.py my_action actionlib/Test")

    action = DynamicAction(args[2])

    app = AXServerApp(action, args[1])
    app.MainLoop()
    rospy.signal_shutdown('GUI shutdown')
