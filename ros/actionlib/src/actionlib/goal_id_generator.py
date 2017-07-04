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

# Author: Alexander Sorokin. 
# Based on C++ goal_id_generator.h/cpp

import rospy

import sys

from actionlib_msgs.msg import GoalID
import threading

global s_goalcount_lock
global s_goalcount
s_goalcount_lock = threading.Lock();
s_goalcount = 0


class GoalIDGenerator:


    def __init__(self,name=None):
        """
        * Create a generator that prepends the fully qualified node name to the Goal ID
        * \param name Unique name to prepend to the goal id. This will
        *             generally be a fully qualified node name.
        """
        if name is not None:
            self.set_name(name)
        else:
            self.set_name(rospy.get_name());


    def set_name(self,name):
        """
        * \param name Set the name to prepend to the goal id. This will
        *             generally be a fully qualified node name.
        """
        self.name=name;

      

    def generate_ID(self):
        """
        * \brief Generates a unique ID
        * \return A unique GoalID for this action
        """
        id = GoalID();
        cur_time = rospy.Time.now();
        ss = self.name +  "-";
        global s_goalcount_lock
        global s_goalcount
        with s_goalcount_lock:
            s_goalcount += 1
            ss += str(s_goalcount) + "-";
        ss +=  str(cur_time.secs) + "." + str(cur_time.nsecs);

        id.id = ss;
        id.stamp = cur_time;
        return id;
