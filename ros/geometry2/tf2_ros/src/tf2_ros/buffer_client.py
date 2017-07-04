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
PKG = 'tf2_ros'
import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib
import tf2_py as tf2
import tf2_ros

from tf2_msgs.msg import LookupTransformAction, LookupTransformGoal
from actionlib_msgs.msg import GoalStatus

class BufferClient(tf2_ros.BufferInterface):
    def __init__(self, ns, check_frequency = 10.0, timeout_padding = rospy.Duration.from_sec(2.0)):
        tf2_ros.BufferInterface.__init__(self)
        self.client = actionlib.SimpleActionClient(ns, LookupTransformAction)
        self.check_frequency = check_frequency
        self.timeout_padding = timeout_padding

    def wait_for_server(self, timeout = rospy.Duration()):
        return self.client.wait_for_server(timeout)

    # lookup, simple api 
    def lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
        goal = LookupTransformGoal()
        goal.target_frame = target_frame;
        goal.source_frame = source_frame;
        goal.source_time = time;
        goal.timeout = timeout;
        goal.advanced = False;

        return self.__process_goal(goal)

    # lookup, advanced api 
    def lookup_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout=rospy.Duration(0.0)):
        goal = LookupTransformGoal()
        goal.target_frame = target_frame;
        goal.source_frame = source_frame;
        goal.source_time = source_time;
        goal.timeout = timeout;
        goal.target_time = target_time;
        goal.fixed_frame = fixed_frame;
        goal.advanced = True;

        return self.__process_goal(goal)

    # can, simple api
    def can_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
        try:
            self.lookup_transform(target_frame, source_frame, time, timeout)
            return True
        except tf2.TransformException:
            return False

    
    # can, advanced api
    def can_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout=rospy.Duration(0.0)):
        try:
            self.lookup_transform_full(target_frame, target_time, source_frame, source_time, fixed_frame, timeout)
            return True
        except tf2.TransformException:
            return False

    def __is_done(self, state):
        if state == GoalStatus.REJECTED or state == GoalStatus.ABORTED or \
           state == GoalStatus.RECALLED or state == GoalStatus.PREEMPTED or \
           state == GoalStatus.SUCCEEDED or state == GoalStatus.LOST:
            return True
        return False

    def __process_goal(self, goal):
        self.client.send_goal(goal)
        r = rospy.Rate(self.check_frequency)
        timed_out = False
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and not self.__is_done(self.client.get_state()) and not timed_out:
            if rospy.Time.now() > start_time + goal.timeout + self.timeout_padding:
                timed_out = True
            r.sleep()

        #This shouldn't happen, but could in rare cases where the server hangs
        if timed_out:
            self.client.cancel_goal()
            raise tf2.TimeoutException("The LookupTransform goal sent to the BufferServer did not come back in the specified time. Something is likely wrong with the server")

        if self.client.get_state() != GoalStatus.SUCCEEDED:
            raise tf2.TimeoutException("The LookupTransform goal sent to the BufferServer did not come back with SUCCEEDED status. Something is likely wrong with the server.")

        return self.__process_result(self.client.get_result())

    def __process_result(self, result):
        if result == None or result.error == None:
            raise tf2.TransformException("The BufferServer returned None for result or result.error!  Something is likely wrong with the server.")
        if result.error.error != result.error.NO_ERROR:
            if result.error.error == result.error.LOOKUP_ERROR:
                raise tf2.LookupException(result.error.error_string)
            if result.error.error == result.error.CONNECTIVITY_ERROR:
                raise tf2.ConnectivityException(result.error.error_string)
            if result.error.error == result.error.EXTRAPOLATION_ERROR:
                raise tf2.ExtrapolationException(result.error.error_string)
            if result.error.error == result.error.INVALID_ARGUMENT_ERROR:
                raise tf2.InvalidArgumentException(result.error.error_string)
            if result.error.error == result.error.TIMEOUT_ERROR:
                raise tf2.TimeoutException(result.error.error_string)

            raise tf2.TransformException(result.error.error_string)

        return result.transform

