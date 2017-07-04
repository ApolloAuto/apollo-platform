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

import actionlib
import actionlib_msgs.msg 
from actionlib import goal_id_generator 


class StatusTracker:
    """
    * @class StatusTracker
    * @brief A class for storing the status of each goal the action server
    * is currently working on
    """

    def __init__(self, goal_id=None, status=None, goal=None):
        """
        @brief create status tracker. Either pass goal_id and status OR goal
        """
        self.goal = None ;
        self.handle_tracker = None;
        self.status = actionlib_msgs.msg.GoalStatus();

        self.handle_destruction_time = rospy.Time();

        self.id_generator = goal_id_generator.GoalIDGenerator();

        if goal_id:
            #set the goal id and status appropriately
            self.status.goal_id = goal_id;
            self.status.status = status;
        else:
            self.goal = goal
            self.status.goal_id = goal.goal_id;

            #initialize the status of the goal to pending
            self.status.status = actionlib_msgs.msg.GoalStatus.PENDING;

            #if the goal id is zero, then we need to make up an id for the goal
            if self.status.goal_id.id == "":
                self.status.goal_id = self.id_generator.generate_ID();

            #if the timestamp of the goal is zero, then we'll set it to now()
            if self.status.goal_id.stamp == rospy.Time():
                self.status.goal_id.stamp = rospy.Time.now();
