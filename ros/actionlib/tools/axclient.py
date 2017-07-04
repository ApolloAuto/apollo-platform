#! /usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#*
#* Author: Eitan Marder-Eppstein
#***********************************************************
"""
usage: %prog /action_name action_type
"""

PKG='actionlib'

import roslib.message

from optparse import OptionParser
import wx
import sys
import rospy
import actionlib
import time
import threading
import socket
import rostopic
from cStringIO import StringIO
from library import *
from dynamic_action import DynamicAction
from actionlib_msgs.msg import GoalStatus

class AXClientApp(wx.App):
    def __init__(self, action_type, action_name):
        self.action_type = action_type
        wx.App.__init__(self)

        self.client = actionlib.SimpleActionClient(action_name, self.action_type.action)
        self.condition = threading.Condition()
        self.goal_msg = None
        self.execute_type = None

    def set_status(self, label, color):
        self.status_bg.SetBackgroundColour(color)
        self.status.SetLabel(label)

    def set_cancel_button(self, enabled):
        if enabled:
            self.cancel_goal.Enable()
        else:
            self.cancel_goal.Disable()


    def set_server_status(self, label, color, enabled):
        self.server_status_bg.SetBackgroundColour(color)
        self.server_status.SetLabel(label)
        if enabled:
            self.send_goal.Enable()
        else:
            self.send_goal.Disable()

    def server_check(self, event):
        TIMEOUT = 0.01
        if self.client.wait_for_server(rospy.Duration.from_sec(TIMEOUT)):
            wx.CallAfter(self.set_server_status, "Connected to server",
                         wx.Colour(192, 252, 253), True)
        else:
            wx.CallAfter(self.set_server_status, "Disconnected from server",
                         wx.Colour(200, 0, 0), False)

    def on_cancel(self, event):
        #we'll cancel the current goal
        self.client.cancel_goal()
        self.set_status("Canceling goal", wx.Colour(211, 34, 243))

    def on_goal(self, event):
        try:
            self.goal_msg = yaml_msg_str(self.action_type.goal,
                                         self.goal.GetValue())
            buff = StringIO()
            self.goal_msg.serialize(buff)

            #send the goal to the action server and register the relevant
            #callbacks
            self.client.send_goal(self.goal_msg, self.done_cb, self.active_cb,
                                  self.feedback_cb)
            self.set_status("Goal is pending", wx.Colour(255, 174, 59))
            self.set_cancel_button(True)

        except roslib.message.SerializationError, e:
            self.goal_msg = None
            wx.MessageBox(str(e), "Error serializing goal", wx.OK)

    def set_result(self, result):
        try:
            self.result.SetValue(to_yaml(result))
        except UnicodeDecodeError:
            self.result.SetValue("Cannot display result due to unprintable characters")

    def status_gui(self, status):
        return {GoalStatus.PENDING: ['PENDING', wx.Colour(255, 174, 59)],
                GoalStatus.ACTIVE: ['ACTIVE', wx.Colour(0, 255, 0)],
                GoalStatus.PREEMPTED: ['PREEMPTED', wx.Colour(255,252,16)],
                GoalStatus.SUCCEEDED: ['SUCCEEDED',wx.Colour(38,250,253)],
                GoalStatus.ABORTED: ['ABORTED',wx.Colour(200,0,0)],
                GoalStatus.REJECTED: ['REJECTED',wx.Colour(253,38,159)],
                GoalStatus.PREEMPTING: ['PREEMPTING',wx.Colour(253,38,159)],
                GoalStatus.RECALLING: ['RECALLING',wx.Colour(230,38,253)],
                GoalStatus.RECALLED: ['RECALLED',wx.Colour(230,38,253)],
                GoalStatus.LOST: ['LOST',wx.Colour(255,0,0)]}[status]

    def done_cb(self, state, result):
        status_string, status_color = self.status_gui(state)
        wx.CallAfter(self.set_status, ''.join(["Goal finished with status: ",
                                               status_string]), status_color)
        wx.CallAfter(self.set_result, result)
        wx.CallAfter(self.set_cancel_button, False)

    def active_cb(self):
        wx.CallAfter(self.set_status, "Goal is active", wx.Colour(0,200,0))

    def set_feedback(self, feedback):
        try:
            self.feedback.SetValue(to_yaml(feedback))
        except UnicodeDecodeError:
            self.feedback.SetValue("Cannot display feedback due to unprintable characters")

    def feedback_cb(self, feedback):
        wx.CallAfter(self.set_feedback, feedback)

    def OnQuit(self):
        self.server_check_timer.Stop()

    def OnInit(self):

        self.frame = wx.Frame(None, -1, self.action_type.name + ' GUI Client')

        self.sz = wx.BoxSizer(wx.VERTICAL)

        tmp_goal = self.action_type.goal()

        self.goal = wx.TextCtrl(self.frame, -1, style=wx.TE_MULTILINE)
        self.goal.SetValue(to_yaml(tmp_goal))
        self.goal_st_bx = wx.StaticBox(self.frame, -1, "Goal")
        self.goal_st = wx.StaticBoxSizer(self.goal_st_bx, wx.VERTICAL)
        self.goal_st.Add(self.goal, 1, wx.EXPAND)

        self.feedback = wx.TextCtrl(self.frame, -1, style=(wx.TE_MULTILINE |
                                                           wx.TE_READONLY))
        self.feedback_st_bx = wx.StaticBox(self.frame, -1, "Feedback")
        self.feedback_st = wx.StaticBoxSizer(self.feedback_st_bx, wx.VERTICAL)
        self.feedback_st.Add(self.feedback, 1, wx.EXPAND)

        self.result = wx.TextCtrl(self.frame, -1, style=(wx.TE_MULTILINE |
                                                         wx.TE_READONLY))
        self.result_st_bx = wx.StaticBox(self.frame, -1, "Result")
        self.result_st = wx.StaticBoxSizer(self.result_st_bx, wx.VERTICAL)
        self.result_st.Add(self.result, 1, wx.EXPAND)

        self.send_goal = wx.Button(self.frame, -1, label="SEND GOAL")
        self.send_goal.Bind(wx.EVT_BUTTON, self.on_goal)
        self.send_goal.Disable()

        self.cancel_goal = wx.Button(self.frame, -1, label="CANCEL GOAL")
        self.cancel_goal.Bind(wx.EVT_BUTTON, self.on_cancel)
        self.cancel_goal.Disable()

        self.status_bg = wx.Panel(self.frame, -1)
        self.status_bg.SetBackgroundColour(wx.Colour(200,0,0))
        self.status = wx.StaticText(self.status_bg, -1, label="No Goal")

        self.server_status_bg = wx.Panel(self.frame, -1)
        self.server_status_bg.SetBackgroundColour(wx.Colour(200,0,0))
        self.server_status = wx.StaticText(self.server_status_bg, -1, label="Disconnected from server.")

        self.sz.Add(self.goal_st, 1, wx.EXPAND)
        self.sz.Add(self.feedback_st, 1, wx.EXPAND)
        self.sz.Add(self.result_st, 1, wx.EXPAND)
        self.sz.Add(self.send_goal, 0, wx.EXPAND)
        self.sz.Add(self.cancel_goal, 0, wx.EXPAND)
        self.sz.Add(self.status_bg, 0, wx.EXPAND)
        self.sz.Add(self.server_status_bg, 0, wx.EXPAND)

        self.frame.SetSizer(self.sz)

        self.server_check_timer = wx.Timer(self.frame)
        self.frame.Bind(wx.EVT_TIMER, self.server_check,
                        self.server_check_timer)
        self.server_check_timer.Start(1000)

        self.sz.Layout()
        self.frame.Show()

        return True




def main():
    rospy.init_node('axclient', anonymous=True)

    parser = OptionParser(__doc__.strip())
#    parser.add_option("-t","--test",action="store_true", dest="test",default=False,
#                      help="A testing flag")
#  parser.add_option("-v","--var",action="store",type="string", dest="var",default="blah")

    (options, args) = parser.parse_args(rospy.myargv())

    if (len(args) == 2):
        # get action type via rostopic
        topic_type = rostopic._get_topic_type("%s/goal"%args[1])[0]
        # remove "Goal" string from action type
        assert("Goal" in topic_type)
        topic_type = topic_type[0:len(topic_type)-4]
    elif (len(args) == 3):
        topic_type = args[2]
        print(topic_type)
        assert("Action" in topic_type)
    else:
        parser.error("You must specify the action topic name (and optionally type) Eg: ./axclient.py action_topic actionlib/TwoIntsAction ")

    action = DynamicAction(topic_type)
    app = AXClientApp(action, args[1])
    app.MainLoop()
    app.OnQuit()
    rospy.signal_shutdown('GUI shutdown')


if __name__ == '__main__':
    main()
