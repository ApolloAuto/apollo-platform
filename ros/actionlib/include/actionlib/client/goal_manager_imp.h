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

/* This file has the template implementation for GoalHandle. It should be included with the
 * class definition.
 */

namespace actionlib
{

template<class ActionSpec>
void GoalManager<ActionSpec>::registerSendGoalFunc(SendGoalFunc send_goal_func)
{
  send_goal_func_ = send_goal_func;
}

template<class ActionSpec>
void GoalManager<ActionSpec>::registerCancelFunc(CancelFunc cancel_func)
{
  cancel_func_ = cancel_func;
}


template<class ActionSpec>
ClientGoalHandle<ActionSpec> GoalManager<ActionSpec>::initGoal(const Goal& goal,
                                                               TransitionCallback transition_cb,
                                                               FeedbackCallback feedback_cb )
{
  ActionGoalPtr action_goal(new ActionGoal);
  action_goal->header.stamp = ros::Time::now();
  action_goal->goal_id = id_generator_.generateID();
  action_goal->goal = goal;

  if (send_goal_func_)
    send_goal_func_(action_goal);
  else
    ROS_WARN_NAMED("actionlib", "Possible coding error: send_goal_func_ set to NULL. Not going to send goal");

  typedef CommStateMachine<ActionSpec> CommStateMachineT;
  boost::shared_ptr<CommStateMachineT> comm_state_machine(new CommStateMachineT(action_goal, transition_cb, feedback_cb));

  boost::recursive_mutex::scoped_lock lock(list_mutex_);
  typename ManagedListT::Handle list_handle = list_.add(comm_state_machine, boost::bind(&GoalManagerT::listElemDeleter, this, _1), guard_);

  return GoalHandleT(this, list_handle, guard_);
}

template<class ActionSpec>
void GoalManager<ActionSpec>::listElemDeleter(typename ManagedListT::iterator it)
{
  assert(guard_);
  DestructionGuard::ScopedProtector protector(*guard_);
  if (!protector.isProtected())
  {
    ROS_ERROR_NAMED("actionlib", "This action client associated with the goal handle has already been destructed. Not going to try delete the CommStateMachine associated with this goal");
    return;
  }

  ROS_DEBUG_NAMED("actionlib", "About to erase CommStateMachine");
  boost::recursive_mutex::scoped_lock lock(list_mutex_);
  list_.erase(it);
  ROS_DEBUG_NAMED("actionlib", "Done erasing CommStateMachine");
}

template<class ActionSpec>
void GoalManager<ActionSpec>::updateStatuses(const actionlib_msgs::GoalStatusArrayConstPtr& status_array)
{
  boost::recursive_mutex::scoped_lock lock(list_mutex_);
  typename ManagedListT::iterator it = list_.begin();

  while (it != list_.end())
  {
    GoalHandleT gh(this, it.createHandle(), guard_);
    (*it)->updateStatus(gh, status_array);
    ++it;
  }
}

template<class ActionSpec>
void GoalManager<ActionSpec>::updateFeedbacks(const ActionFeedbackConstPtr& action_feedback)
{
  boost::recursive_mutex::scoped_lock lock(list_mutex_);
  typename ManagedListT::iterator it = list_.begin();

  while (it != list_.end())
  {
    GoalHandleT gh(this, it.createHandle(), guard_);
    (*it)->updateFeedback(gh, action_feedback);
    ++it;
  }
}

template<class ActionSpec>
void GoalManager<ActionSpec>::updateResults(const ActionResultConstPtr& action_result)
{
  boost::recursive_mutex::scoped_lock lock(list_mutex_);
  typename ManagedListT::iterator it = list_.begin();

  while (it != list_.end())
  {
    GoalHandleT gh(this, it.createHandle(), guard_);
    (*it)->updateResult(gh, action_result);
    ++it;
  }
}


}
