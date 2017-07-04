/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef ACTIONLIB_SERVER_GOAL_HANDLE_H_
#define ACTIONLIB_SERVER_GOAL_HANDLE_H_

#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib/action_definition.h>
#include <actionlib/server/status_tracker.h>
#include <actionlib/destruction_guard.h>
#include <boost/shared_ptr.hpp>

namespace actionlib {
  //forward declaration of ActionServerBase
  template <class ActionSpec>
  class ActionServerBase;

  /**
   * @class ServerGoalHandle
   * @brief Encapsulates a state machine for a given goal that the user can
   * trigger transisions on. All ROS interfaces for the goal are managed by
   * the ActionServer to lessen the burden on the user.
   */
  template <class ActionSpec>
  class ServerGoalHandle {
    private:
      //generates typedefs that we'll use to make our lives easier
      ACTION_DEFINITION(ActionSpec);

    public:
      /**
       * @brief  Default constructor for a ServerGoalHandle
       */
      ServerGoalHandle();

      /**
       * @brief  Copy constructor for a ServerGoalHandle
       * @param gh The goal handle to copy 
       */
      ServerGoalHandle(const ServerGoalHandle& gh);

      /** @brief  Accept the goal referenced by the goal handle. This will
       * transition to the ACTIVE state or the PREEMPTING state depending
       * on whether a cancel request has been received for the goal
       * @param text Optionally, any text message about the status change being made that should be passed to the client
       */
      void setAccepted(const std::string& text = std::string(""));

      /**
       * @brief  Set the status of the goal associated with the ServerGoalHandle to RECALLED or PREEMPTED
       * depending on what the current status of the goal is
       * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
       * @param text Optionally, any text message about the status change being made that should be passed to the client
       */
      void setCanceled(const Result& result = Result(), const std::string& text = std::string(""));

      /**
       * @brief  Set the status of the goal associated with the ServerGoalHandle to rejected
       * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
       * @param text Optionally, any text message about the status change being made that should be passed to the client
       */
      void setRejected(const Result& result = Result(), const std::string& text = std::string(""));

      /**
       * @brief  Set the status of the goal associated with the ServerGoalHandle to aborted
       * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
       * @param text Optionally, any text message about the status change being made that should be passed to the client
       */
      void setAborted(const Result& result = Result(), const std::string& text = std::string(""));

      /**
       * @brief  Set the status of the goal associated with the ServerGoalHandle to succeeded
       * @param  result Optionally, the user can pass in a result to be sent to any clients of the goal
       * @param text Optionally, any text message about the status change being made that should be passed to the client
       */
      void setSucceeded(const Result& result = Result(), const std::string& text = std::string(""));

      /**
       * @brief  Send feedback to any clients of the goal associated with this ServerGoalHandle
       * @param feedback The feedback to send to the client
       */
      void publishFeedback(const Feedback& feedback);

      /**
       * @brief Determine if the goal handle is valid (tracking a valid goal,
       * and associated with a valid action server). If the handle is valid, it
       * means that the accessors \ref getGoal, \ref getGoalID, etc, can be
       * called without generating errors.
       *
       * @return True if valid, False if invalid
       */
      bool isValid() const;

      /**
       * @brief  Accessor for the goal associated with the ServerGoalHandle
       * @return A shared_ptr to the goal object
       */
      boost::shared_ptr<const Goal> getGoal() const;

      /**
       * @brief  Accessor for the goal id associated with the ServerGoalHandle
       * @return The goal id
       */
      actionlib_msgs::GoalID getGoalID() const;

      /**
       * @brief  Accessor for the status associated with the ServerGoalHandle
       * @return The goal status
       */
      actionlib_msgs::GoalStatus getGoalStatus() const;

      /**
       * @brief  Equals operator for a ServerGoalHandle
       * @param gh The goal handle to copy 
       */
      ServerGoalHandle& operator=(const ServerGoalHandle& gh);

      /**
       * @brief  Equals operator for ServerGoalHandles
       * @param other The ServerGoalHandle to compare to
       * @return True if the ServerGoalHandles refer to the same goal, false otherwise
       */
      bool operator==(const ServerGoalHandle& other) const;

      /**
       * @brief  != operator for ServerGoalHandles
       * @param other The ServerGoalHandle to compare to
       * @return True if the ServerGoalHandles refer to different goals, false otherwise
       */
      bool operator!=(const ServerGoalHandle& other) const;

    private:
      /**
       * @brief  A private constructor used by the ActionServer to initialize a ServerGoalHandle
       */
      ServerGoalHandle(typename std::list<StatusTracker<ActionSpec> >::iterator status_it,
          ActionServerBase<ActionSpec>* as, boost::shared_ptr<void> handle_tracker, boost::shared_ptr<DestructionGuard> guard);

      /**
       * @brief  A private method to set status to PENDING or RECALLING
       * @return True if the cancel request should be passed on to the user, false otherwise
       */
      bool setCancelRequested();

      typename std::list<StatusTracker<ActionSpec> >::iterator status_it_;
      boost::shared_ptr<const ActionGoal> goal_;
      ActionServerBase<ActionSpec>* as_;
      boost::shared_ptr<void> handle_tracker_;
      boost::shared_ptr<DestructionGuard> guard_;
      friend class ActionServerBase<ActionSpec>;
  };

};

//include the implementation
#include <actionlib/server/server_goal_handle_imp.h>

#endif
