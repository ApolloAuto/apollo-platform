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

#ifndef ACTIONLIB_CLIENT_COMM_STATE_H_
#define ACTIONLIB_CLIENT_COMM_STATE_H_

#include <string>
#include "ros/console.h"

namespace actionlib
{

/**
 * \brief Thin wrapper around an enum in order to help interpret the state of the communication state machine
 **/
class CommState
{
public:

  //! \brief Defines the various states the Communication State Machine can be in
  enum StateEnum
  {
    WAITING_FOR_GOAL_ACK    = 0,
    PENDING                 = 1,
    ACTIVE                  = 2,
    WAITING_FOR_RESULT      = 3,
    WAITING_FOR_CANCEL_ACK  = 4,
    RECALLING               = 5,
    PREEMPTING              = 6,
    DONE                    = 7
  } ;

  CommState(const StateEnum& state) : state_(state) { }

  inline bool operator==(const CommState& rhs) const
  {
    return (state_ == rhs.state_) ;
  }

  inline bool operator==(const CommState::StateEnum& rhs) const
  {
    return (state_ == rhs);
  }

  inline bool operator!=(const CommState::StateEnum& rhs) const
  {
    return !(*this == rhs);
  }

  inline bool operator!=(const CommState& rhs) const
  {
    return !(*this == rhs);
  }

  std::string toString() const
  {
    switch(state_)
    {
      case WAITING_FOR_GOAL_ACK:
        return "WAITING_FOR_GOAL_ACK";
      case PENDING:
        return "PENDING";
      case ACTIVE:
        return "ACTIVE";
      case WAITING_FOR_RESULT:
        return "WAITING_FOR_RESULT";
      case WAITING_FOR_CANCEL_ACK:
        return "WAITING_FOR_CANCEL_ACK";
      case RECALLING:
        return "RECALLING";
      case PREEMPTING:
        return "PREEMPTING";
      case DONE:
        return "DONE";
      default:
        ROS_ERROR_NAMED("actionlib", "BUG: Unhandled CommState: %u", state_);
        break;
    }
    return "BUG-UNKNOWN";
  }

  StateEnum state_;
private:
  CommState();

} ;

}

#endif
