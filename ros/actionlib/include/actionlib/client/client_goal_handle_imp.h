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

/* This file has the template implementation for ClientGoalHandle. It should be included with the
 * class definition.
 */

namespace actionlib
{

template<class ActionSpec>
ClientGoalHandle<ActionSpec>::ClientGoalHandle()
{
  gm_ = NULL;
  active_ = false;
}

template<class ActionSpec>
ClientGoalHandle<ActionSpec>::~ClientGoalHandle()
{
  reset();
}

template<class ActionSpec>
ClientGoalHandle<ActionSpec>::ClientGoalHandle(GoalManagerT* gm, typename ManagedListT::Handle handle,
                                               const boost::shared_ptr<DestructionGuard>& guard)
{
  gm_ = gm;
  active_ = true;
  list_handle_ = handle;
  guard_ = guard;
}

template<class ActionSpec>
void ClientGoalHandle<ActionSpec>::reset()
{
  if (active_)
  {
    DestructionGuard::ScopedProtector protector(*guard_);
    if (!protector.isProtected())
    {
      ROS_ERROR_NAMED("actionlib", "This action client associated with the goal handle has already been destructed. Ignoring this reset() call");
      return;
    }

    boost::recursive_mutex::scoped_lock lock(gm_->list_mutex_);
    list_handle_.reset();
    active_ = false;
    gm_ = NULL;
  }
}

template<class ActionSpec>
bool ClientGoalHandle<ActionSpec>::isExpired() const
{
  return !active_;
}


template<class ActionSpec>
CommState ClientGoalHandle<ActionSpec>::getCommState()
{
  if (!active_)
  {
    ROS_ERROR_NAMED("actionlib", "Trying to getCommState on an inactive ClientGoalHandle. You are incorrectly using a ClientGoalHandle");
    return CommState(CommState::DONE);
  }

  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected())
  {
    ROS_ERROR_NAMED("actionlib", "This action client associated with the goal handle has already been destructed. Ignoring this getCommState() call");
    return CommState(CommState::DONE);
  }

  assert(gm_);

  boost::recursive_mutex::scoped_lock lock(gm_->list_mutex_);
  return list_handle_.getElem()->getCommState();
}

template<class ActionSpec>
TerminalState ClientGoalHandle<ActionSpec>::getTerminalState()
{

  if (!active_)
  {
    ROS_ERROR_NAMED("actionlib", "Trying to getTerminalState on an inactive ClientGoalHandle. You are incorrectly using a ClientGoalHandle");
    return TerminalState(TerminalState::LOST);
  }

  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected())
  {
    ROS_ERROR_NAMED("actionlib", "This action client associated with the goal handle has already been destructed. Ignoring this getTerminalState() call");
    return TerminalState(TerminalState::LOST);
  }

  assert(gm_);

  boost::recursive_mutex::scoped_lock lock(gm_->list_mutex_);
  CommState comm_state_ = list_handle_.getElem()->getCommState();
  if (comm_state_ != CommState::DONE)
    ROS_WARN_NAMED("actionlib", "Asking for the terminal state when we're in [%s]", comm_state_.toString().c_str());

  actionlib_msgs::GoalStatus goal_status = list_handle_.getElem()->getGoalStatus();

  switch (goal_status.status)
  {
    case actionlib_msgs::GoalStatus::PENDING:
    case actionlib_msgs::GoalStatus::ACTIVE:
    case actionlib_msgs::GoalStatus::PREEMPTING:
    case actionlib_msgs::GoalStatus::RECALLING:
      ROS_ERROR_NAMED("actionlib", "Asking for terminal state, but latest goal status is %u", goal_status.status); return TerminalState(TerminalState::LOST, goal_status.text);
    case actionlib_msgs::GoalStatus::PREEMPTED: return TerminalState(TerminalState::PREEMPTED, goal_status.text);
    case actionlib_msgs::GoalStatus::SUCCEEDED: return TerminalState(TerminalState::SUCCEEDED, goal_status.text);
    case actionlib_msgs::GoalStatus::ABORTED:   return TerminalState(TerminalState::ABORTED, goal_status.text);
    case actionlib_msgs::GoalStatus::REJECTED:  return TerminalState(TerminalState::REJECTED, goal_status.text);
    case actionlib_msgs::GoalStatus::RECALLED:  return TerminalState(TerminalState::RECALLED, goal_status.text);
    case actionlib_msgs::GoalStatus::LOST:      return TerminalState(TerminalState::LOST, goal_status.text);
    default:
      ROS_ERROR_NAMED("actionlib", "Unknown goal status: %u", goal_status.status); break;
  }

  ROS_ERROR_NAMED("actionlib", "Bug in determining terminal state");
  return TerminalState(TerminalState::LOST, goal_status.text);
}

template<class ActionSpec>
typename ClientGoalHandle<ActionSpec>::ResultConstPtr ClientGoalHandle<ActionSpec>::getResult()
{
  if (!active_)
    ROS_ERROR_NAMED("actionlib", "Trying to getResult on an inactive ClientGoalHandle. You are incorrectly using a ClientGoalHandle");
  assert(gm_);

  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected())
  {
    ROS_ERROR_NAMED("actionlib", "This action client associated with the goal handle has already been destructed. Ignoring this getResult() call");
    return typename ClientGoalHandle<ActionSpec>::ResultConstPtr() ;
  }

  boost::recursive_mutex::scoped_lock lock(gm_->list_mutex_);
  return list_handle_.getElem()->getResult();
}

template<class ActionSpec>
void ClientGoalHandle<ActionSpec>::resend()
{
  if (!active_)
    ROS_ERROR_NAMED("actionlib", "Trying to resend() on an inactive ClientGoalHandle. You are incorrectly using a ClientGoalHandle");

  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected())
  {
    ROS_ERROR_NAMED("actionlib", "This action client associated with the goal handle has already been destructed. Ignoring this resend() call");
    return;
  }

  assert(gm_);

  boost::recursive_mutex::scoped_lock lock(gm_->list_mutex_);

  ActionGoalConstPtr action_goal = list_handle_.getElem()->getActionGoal();

  if (!action_goal)
    ROS_ERROR_NAMED("actionlib", "BUG: Got a NULL action_goal");

  if (gm_->send_goal_func_)
    gm_->send_goal_func_(action_goal);
}

template<class ActionSpec>
void ClientGoalHandle<ActionSpec>::cancel()
{
  if (!active_)
  {
    ROS_ERROR_NAMED("actionlib", "Trying to cancel() on an inactive ClientGoalHandle. You are incorrectly using a ClientGoalHandle");
    return;
  }
  assert(gm_);

  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected())
  {
    ROS_ERROR_NAMED("actionlib", "This action client associated with the goal handle has already been destructed. Ignoring this call");
    return;
  }

  boost::recursive_mutex::scoped_lock lock(gm_->list_mutex_);

  switch(list_handle_.getElem()->getCommState().state_)
  {
    case CommState::WAITING_FOR_GOAL_ACK:
    case CommState::PENDING:
    case CommState::ACTIVE:
    case CommState::WAITING_FOR_CANCEL_ACK:
      break; // Continue standard processing
    case CommState::WAITING_FOR_RESULT:
    case CommState::RECALLING:
    case CommState::PREEMPTING:
    case CommState::DONE:
      ROS_DEBUG_NAMED("actionlib", "Got a cancel() request while in state [%s], so ignoring it", list_handle_.getElem()->getCommState().toString().c_str());
      return;
    default:
      ROS_ERROR_NAMED("actionlib", "BUG: Unhandled CommState: %u", list_handle_.getElem()->getCommState().state_);
      return;
  }

  ActionGoalConstPtr action_goal = list_handle_.getElem()->getActionGoal();

  actionlib_msgs::GoalID cancel_msg;
  cancel_msg.stamp = ros::Time(0,0);
  cancel_msg.id = list_handle_.getElem()->getActionGoal()->goal_id.id;

  if (gm_->cancel_func_)
    gm_->cancel_func_(cancel_msg);

  list_handle_.getElem()->transitionToState(*this, CommState::WAITING_FOR_CANCEL_ACK);
}

template<class ActionSpec>
bool ClientGoalHandle<ActionSpec>::operator==(const ClientGoalHandle<ActionSpec>& rhs) const
{
  // Check if both are inactive
  if (!active_ && !rhs.active_)
    return true;

  // Check if one or the other is inactive
  if (!active_ || !rhs.active_)
    return false;

  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected())
  {
    ROS_ERROR_NAMED("actionlib", "This action client associated with the goal handle has already been destructed. Ignoring this operator==() call");
    return false;
  }

  return (list_handle_ == rhs.list_handle_) ;
}

template<class ActionSpec>
bool ClientGoalHandle<ActionSpec>::operator!=(const ClientGoalHandle<ActionSpec>& rhs) const
{
  return !(*this==rhs);
}

}
