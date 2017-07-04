/*********************************************************************
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#ifndef ACTIONLIB_CLIENT_GOAL_STATUS_H_
#define ACTIONLIB_CLIENT_GOAL_STATUS_H_

#include <string>

#include "actionlib/GoalStatus.h"

namespace actionlib
{

/**
 * \brief Thin wrapper around an enum in order to help interpret the client-side status of a goal request
 * The possible states are defined the ClientGoalStatus::StateEnum. They are also defined in StateEnum.
 * we can also get there from \link ClientGoalStatus::StateEnum here \endlink
 **/
class ClientGoalStatus
{
public:
   //! \brief Defines the various states the Goal can be in, as perceived by the client
  enum StateEnum
  {
    PENDING,    //!< The goal has yet to be processed by the action server
    ACTIVE,     //!< The goal is currently being processed by the action server
    PREEMPTED,  //!< The goal was preempted by either another goal, or a preempt message being sent to the action server
    SUCCEEDED,  //!< The goal was achieved successfully by the action server
    ABORTED,    //!< The goal was aborted by the action server
    REJECTED,   //!< The ActionServer refused to start processing the goal, possibly because a goal is infeasible
    LOST        //!< The goal was sent by the ActionClient, but disappeared due to some communication error
  } ;

  ClientGoalStatus(StateEnum state)
  {
    state_ = state;
  }

  /**
   * \brief Build a ClientGoalStatus from a GoalStatus.
   * Note that the only GoalStatuses that can be converted into a
   * ClientGoalStatus are {PREEMPTED, SUCCEEDED, ABORTED, REJECTED}
   * \param goal_status The GoalStatus msg that we want to convert
   */
  ClientGoalStatus(const GoalStatus& goal_status)
  {
    fromGoalStatus(goal_status);
  }

  /**
   * \brief Check if the goal is in a terminal state
   * \return TRUE if in PREEMPTED, SUCCEDED, ABORTED, REJECTED, or LOST
   */
  inline bool isDone() const
  {
    if (state_ == PENDING || state_ == ACTIVE)
      return false;
    return true;
  }

  /**
   * \brief Copy the raw enum into the object
   */
  inline const StateEnum& operator=(const StateEnum& state)
  {
    state_ = state;
    return state;
  }

  /**
   * \brief Straightforward enum equality check
   */
  inline bool operator==(const ClientGoalStatus& rhs) const
  {
    return state_ == rhs.state_;
  }

  /**
   * \brief Straightforward enum inequality check
   */
  inline bool operator!=(const ClientGoalStatus& rhs) const
  {
    return !(state_ == rhs.state_);
  }

  /**
   * \brief Store a GoalStatus in a ClientGoalStatus
   * Note that the only GoalStatuses that can be converted into a
   * ClientGoalStatus are {PREEMPTED, SUCCEEDED, ABORTED, REJECTED}
   * \param goal_status The GoalStatus msg that we want to convert
   */
  void fromGoalStatus(const GoalStatus& goal_status)
  {
    switch(goal_status.status)
    {
      case GoalStatus::PREEMPTED:
        state_ = ClientGoalStatus::PREEMPTED; break;
      case GoalStatus::SUCCEEDED:
        state_ = ClientGoalStatus::SUCCEEDED; break;
      case GoalStatus::ABORTED:
        state_ = ClientGoalStatus::ABORTED; break;
      case GoalStatus::REJECTED:
        state_ = ClientGoalStatus::REJECTED; break;
      default:
        state_ = ClientGoalStatus::LOST;
        ROS_ERROR_NAMED("actionlib", "Cannot convert GoalStatus %u to ClientGoalState", goal_status.status); break;
    }
  }

  /**
   * \brief Stringify the enum
   * \return String that has the name of the enum
   */
  std::string toString() const
  {
    switch(state_)
    {
      case PENDING:
        return "PENDING";
      case ACTIVE:
        return "ACTIVE";
      case PREEMPTED:
        return "PREEMPTED";
      case SUCCEEDED:
        return "SUCCEEDED";
      case ABORTED:
        return "ABORTED";
      case REJECTED:
        return "REJECTED";
      case LOST:
        return "LOST";
      default:
        ROS_ERROR_NAMED("actionlib", "BUG: Unhandled ClientGoalStatus");
        break;
    }
    return "BUG-UNKNOWN";
  }

private:
  StateEnum state_;
  ClientGoalStatus(); //!< Need to always specific an initial state. Thus, no empty constructor
};

}

#endif // ACTION_TOOLS_CLIENT_GOAL_STATE_H_
