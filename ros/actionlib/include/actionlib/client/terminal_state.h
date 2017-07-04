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

#ifndef ACTIONLIB_CLIENT_TERMINAL_STATE_H_
#define ACTIONLIB_CLIENT_TERMINAL_STATE_H_

namespace actionlib
{

class TerminalState
{
public:
  enum StateEnum
  {
    RECALLED,
    REJECTED,
    PREEMPTED,
    ABORTED,
    SUCCEEDED,
    LOST
  } ;

  TerminalState(const StateEnum& state, const std::string& text = std::string("")) : state_(state), text_(text) { }

  inline bool operator==(const TerminalState& rhs) const
  {
    return (state_ == rhs.state_) ;
  }

  inline bool operator==(const TerminalState::StateEnum& rhs) const
  {
    return (state_ == rhs);
  }

  inline bool operator!=(const TerminalState::StateEnum& rhs) const
  {
    return !(*this == rhs);
  }

  inline bool operator!=(const TerminalState& rhs) const
  {
    return !(*this == rhs);
  }

  std::string getText() const
  {
    return text_;
  }

  std::string toString() const
  {
    switch(state_)
    {
      case RECALLED:
        return "RECALLED";
      case REJECTED:
        return "REJECTED";
      case PREEMPTED:
        return "PREEMPTED";
      case ABORTED:
        return "ABORTED";
      case SUCCEEDED:
        return "SUCCEEDED";
      case LOST:
        return "LOST";

      default:
        ROS_ERROR_NAMED("actionlib", "BUG: Unhandled TerminalState: %u", state_);
        break;
    }
    return "BUG-UNKNOWN";
  }

  StateEnum state_;
  std::string text_;
private:
  TerminalState();

} ;

}

#endif
