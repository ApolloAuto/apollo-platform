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

import actionlib_msgs.msg

class ServerGoalHandle:
    """
    * @class ServerGoalHandle
    * @brief Encapsulates a state machine for a given goal that the user can
    * trigger transisions on. All ROS interfaces for the goal are managed by
    * the ActionServer to lessen the burden on the user.

    """

    def __init__(self, status_tracker=None, action_server=None, handle_tracker=None):
        """
        A private constructor used by the ActionServer to initialize a ServerGoalHandle.
        @node  The default constructor was not ported.
        """
        self.status_tracker = status_tracker;
        self.action_server = action_server
        self.handle_tracker = handle_tracker;

        if status_tracker:
            self.goal=status_tracker.goal;
        else:
            self.goal=None


    def get_default_result(self):
        return self.action_server.ActionResultType();

    def set_accepted(self, text=""):
      """
      Accept the goal referenced by the goal handle. This will
      transition to the ACTIVE state or the PREEMPTING state depending
      on whether a cancel request has been received for the goal
      """

      rospy.logdebug("Accepting goal, id: %s, stamp: %.2f", self.get_goal_id().id, self.get_goal_id().stamp.to_sec());
      if self.goal:
          with self.action_server.lock:
              status = self.status_tracker.status.status;

              #if we were pending before, then we'll go active
              if(status == actionlib_msgs.msg.GoalStatus.PENDING):
                  self.status_tracker.status.status = actionlib_msgs.msg.GoalStatus.ACTIVE;
                  self.status_tracker.status.text = text
                  self.action_server.publish_status();

              #if we were recalling before, now we'll go to preempting
              elif(status == actionlib_msgs.msg.GoalStatus.RECALLING) :
                  self.status_tracker.status.status = actionlib_msgs.msg.GoalStatus.PREEMPTING;
                  self.status_tracker.status.text = text
                  self.action_server.publish_status();

              else:
                  rospy.logerr("To transition to an active state, the goal must be in a pending or recalling state, it is currently in state: %d",  self.status_tracker.status.status);

      else:
          rospy.logerr("Attempt to set status on an uninitialized ServerGoalHandle");


    def set_canceled(self, result=None, text=""):
        """
        Set the status of the goal associated with the ServerGoalHandle to RECALLED or PREEMPTED
        depending on what the current status of the goal is
        @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
        """
        if not result:
            result=self.get_default_result();

        rospy.logdebug("Setting status to canceled on goal, id: %s, stamp: %.2f", self.get_goal_id().id, self.get_goal_id().stamp.to_sec());

        if(self.goal):
            with self.action_server.lock:
                status = self.status_tracker.status.status;
                if(status == actionlib_msgs.msg.GoalStatus.PENDING or status == actionlib_msgs.msg.GoalStatus.RECALLING):
                    self.status_tracker.status.status = actionlib_msgs.msg.GoalStatus.RECALLED;
                    self.status_tracker.status.text = text
                    #on transition to a terminal state, we'll also set the handle destruction time
                    self.status_tracker.handle_destruction_time = rospy.Time.now()
                    self.action_server.publish_result(self.status_tracker.status, result);
                elif(status == actionlib_msgs.msg.GoalStatus.ACTIVE or status == actionlib_msgs.msg.GoalStatus.PREEMPTING):
                    self.status_tracker.status.status = actionlib_msgs.msg.GoalStatus.PREEMPTED;
                    self.status_tracker.status.text = text
                    #on transition to a terminal state, we'll also set the handle destruction time
                    self.status_tracker.handle_destruction_time = rospy.Time.now()
                    self.action_server.publish_result(self.status_tracker.status, result);

                else:
                    rospy.logerr("To transition to a cancelled state, the goal must be in a pending, recalling, active, or preempting state, it is currently in state: %d",
                                 self.status_tracker.status.status);

        else:
            rospy.logerr("Attempt to set status on an uninitialized ServerGoalHandle");


    def set_rejected(self, result=None, text=""):
        """
        * @brief  Set the status of the goal associated with the ServerGoalHandle to rejected
        * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
        """
        if not result:
            result=self.get_default_result();

        rospy.logdebug("Setting status to rejected on goal, id: %s, stamp: %.2f", self.get_goal_id().id, self.get_goal_id().stamp.to_sec());
        if self.goal :
            with self.action_server.lock:
                status = self.status_tracker.status.status;
                if(status == actionlib_msgs.msg.GoalStatus.PENDING or status == actionlib_msgs.msg.GoalStatus.RECALLING):
                    self.status_tracker.status.status = actionlib_msgs.msg.GoalStatus.REJECTED;
                    self.status_tracker.status.text = text
                    #on transition to a terminal state, we'll also set the handle destruction time
                    self.status_tracker.handle_destruction_time = rospy.Time.now()
                    self.action_server.publish_result(self.status_tracker.status, result);

                else:
                    rospy.logerr("To transition to a rejected state, the goal must be in a pending or recalling state, it is currently in state: %d",
                                 self.status_tracker.status.status);

        else:
            rospy.logerr("Attempt to set status on an uninitialized ServerGoalHandle");


    def set_aborted(self, result=None, text=""):
        """
        Set the status of the goal associated with the ServerGoalHandle to aborted
        @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
        """
        if not result:
            result=self.get_default_result();

        rospy.logdebug("Setting status to aborted on goal, id: %s, stamp: %.2f", self.get_goal_id().id, self.get_goal_id().stamp.to_sec());
        if self.goal:
            with self.action_server.lock:
                status = self.status_tracker.status.status;
                if status == actionlib_msgs.msg.GoalStatus.PREEMPTING or status == actionlib_msgs.msg.GoalStatus.ACTIVE:
                    self.status_tracker.status.status = actionlib_msgs.msg.GoalStatus.ABORTED;
                    self.status_tracker.status.text = text
                    #on transition to a terminal state, we'll also set the handle destruction time
                    self.status_tracker.handle_destruction_time = rospy.Time.now()
                    self.action_server.publish_result(self.status_tracker.status, result);

                else:
                    rospy.logerr("To transition to an aborted state, the goal must be in a preempting or active state, it is currently in state: %d",
                                 status);

        else:
            rospy.logerr("Attempt to set status on an uninitialized ServerGoalHandle");


    def set_succeeded(self,result=None, text=""):
        """
        Set the status of the goal associated with the ServerGoalHandle to succeeded
        @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
        """
        if not result:
            result=self.get_default_result();

        rospy.logdebug("Setting status to succeeded on goal, id: %s, stamp: %.2f",
                       self.get_goal_id().id, self.get_goal_id().stamp.to_sec());
        if self.goal:
            with self.action_server.lock:
                status = self.status_tracker.status.status;
                if status == actionlib_msgs.msg.GoalStatus.PREEMPTING or status == actionlib_msgs.msg.GoalStatus.ACTIVE :
                    self.status_tracker.status.status = actionlib_msgs.msg.GoalStatus.SUCCEEDED;
                    self.status_tracker.status.text = text
                    #on transition to a terminal state, we'll also set the handle destruction time
                    self.status_tracker.handle_destruction_time = rospy.Time.now()
                    self.action_server.publish_result(self.status_tracker.status, result);

                else:
                    rospy.logerr("To transition to a succeeded state, the goal must be in a preempting or active state, it is currently in state: %d",
                                 status);

        else:
            rospy.logerr("Attempt to set status on an uninitialized ServerGoalHandle");


    def publish_feedback(self,feedback):
        """
        Send feedback to any clients of the goal associated with this ServerGoalHandle
        @param feedback The feedback to send to the client
        """
        rospy.logdebug("Publishing feedback for goal, id: %s, stamp: %.2f",
                       self.get_goal_id().id, self.get_goal_id().stamp.to_sec());
        if self.goal:
            with self.action_server.lock:
                self.action_server.publish_feedback(self.status_tracker.status, feedback);
        else:
            rospy.logerr("Attempt to publish feedback on an uninitialized ServerGoalHandle");


    def get_goal(self):
        """
        Accessor for the goal associated with the ServerGoalHandle
        @return A shared_ptr to the goal object
        """
          #if we have a goal that is non-null
        if self.goal:
            #@todo Test that python reference counting automatically handles this.
            #create the deleter for our goal subtype
            #d = EnclosureDeleter(self.goal);
            #weakref.ref(boost::shared_ptr<const Goal>(&(goal_->goal), d);
            return self.goal.goal

        return None


    def get_goal_id(self):
        """
        Accessor for the goal id associated with the ServerGoalHandle
        @return The goal id
        """
        if self.goal:
            with self.action_server.lock:
                return self.status_tracker.status.goal_id;
        else:
            rospy.logerr("Attempt to get a goal id on an uninitialized ServerGoalHandle");
            return actionlib_msgs.msg.GoalID();


    def get_goal_status(self):
        """
        Accessor for the status associated with the ServerGoalHandle
        @return The goal status
        """
        if self.goal:
            with self.action_server.lock:
                return self.status_tracker.status;
        else:
            rospy.logerr("Attempt to get goal status on an uninitialized ServerGoalHandle");
            return actionlib_msgs.msg.GoalStatus();


    def __eq__(self, other):
        """
        Equals operator for ServerGoalHandles
        @param other The ServerGoalHandle to compare to
        @return True if the ServerGoalHandles refer to the same goal, false otherwise
        """

        if( not self.goal or not other.goal):
            return False;
        my_id = self.get_goal_id();
        their_id = other.get_goal_id();
        return my_id.id == their_id.id;

    def __ne__(self, other):
        """
        != operator for ServerGoalHandles
        @param other The ServerGoalHandle to compare to
        @return True if the ServerGoalHandles refer to different goals, false otherwise
        """
        if( not self.goal or not other.goal):
            return True;
        my_id = self.get_goal_id();
        their_id = other.get_goal_id();

        return my_id.id != their_id.id;

    def __hash__(self):
        """
        hash function for ServerGoalHandles
        @return hash of the goal ID
        """
        return hash(self.get_goal_id().id)

    def set_cancel_requested(self):
        """
        A private method to set status to PENDING or RECALLING
        @return True if the cancel request should be passed on to the user, false otherwise
        """
        rospy.logdebug("Transisitoning to a cancel requested state on goal id: %s, stamp: %.2f",
                       self.get_goal_id().id, self.get_goal_id().stamp.to_sec());
        if self.goal:
            with self.action_server.lock:
                status = self.status_tracker.status.status;
                if (status == actionlib_msgs.msg.GoalStatus.PENDING):
                    self.status_tracker.status.status = actionlib_msgs.msg.GoalStatus.RECALLING;
                    self.action_server.publish_status();
                    return True;

                if(status == actionlib_msgs.msg.GoalStatus.ACTIVE):
                    self.status_tracker.status.status = actionlib_msgs.msg.GoalStatus.PREEMPTING;
                    self.action_server.publish_status();
                    return True;

        return False;
