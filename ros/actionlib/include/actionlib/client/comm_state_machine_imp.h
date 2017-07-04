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

/* This file has the template implementation for CommStateMachine. It should be included with the
 * class definition.
 */

namespace actionlib
{

template <class ActionSpec>
CommStateMachine<ActionSpec>::CommStateMachine(const ActionGoalConstPtr& action_goal,
                                               TransitionCallback transition_cb,
                                               FeedbackCallback feedback_cb) : state_(CommState::WAITING_FOR_GOAL_ACK)
{
  assert(action_goal);
  action_goal_ = action_goal;
  transition_cb_ = transition_cb;
  feedback_cb_ = feedback_cb;
  //transitionToState( CommState::WAITING_FOR_GOAL_ACK );
}

template <class ActionSpec>
typename CommStateMachine<ActionSpec>::ActionGoalConstPtr CommStateMachine<ActionSpec>::getActionGoal() const
{
  return action_goal_;
}

template <class ActionSpec>
CommState CommStateMachine<ActionSpec>::getCommState() const
{
  return state_;
}

template <class ActionSpec>
actionlib_msgs::GoalStatus CommStateMachine<ActionSpec>::getGoalStatus() const
{
  return latest_goal_status_;
}

template <class ActionSpec>
typename CommStateMachine<ActionSpec>::ResultConstPtr CommStateMachine<ActionSpec>::getResult() const
{
  ResultConstPtr result;
  if (latest_result_)
  {
    EnclosureDeleter<const ActionResult> d(latest_result_);
    result = ResultConstPtr(&(latest_result_->result), d);
  }
  return result;

}

template <class ActionSpec>
void CommStateMachine<ActionSpec>::setCommState(const CommState::StateEnum& state)
{
  setCommState(CommState(state));
}

template <class ActionSpec>
void CommStateMachine<ActionSpec>::setCommState(const CommState& state)
{
  ROS_DEBUG_NAMED("actionlib", "Transitioning CommState from %s to %s", state_.toString().c_str(), state.toString().c_str());
  state_ = state;
}

template <class ActionSpec>
const actionlib_msgs::GoalStatus* CommStateMachine<ActionSpec>::findGoalStatus(const std::vector<actionlib_msgs::GoalStatus>& status_vec) const
{
  for (unsigned int i=0; i<status_vec.size(); i++)
    if (status_vec[i].goal_id.id == action_goal_->goal_id.id)
      return &status_vec[i];
  return NULL;
}

template <class ActionSpec>
void CommStateMachine<ActionSpec>::updateFeedback(GoalHandleT& gh, const ActionFeedbackConstPtr& action_feedback)
{
  // Check if this feedback is for us
  if (action_goal_->goal_id.id != action_feedback->status.goal_id.id)
    return;

  if (feedback_cb_)
  {
    EnclosureDeleter<const ActionFeedback> d(action_feedback);
    FeedbackConstPtr feedback(&(action_feedback->feedback), d);
    feedback_cb_(gh, feedback);
  }
}

template <class ActionSpec>
void CommStateMachine<ActionSpec>::updateResult(GoalHandleT& gh, const ActionResultConstPtr& action_result)
{
  // Check if this feedback is for us
  if (action_goal_->goal_id.id != action_result->status.goal_id.id)
    return;
  latest_goal_status_ = action_result->status;
  latest_result_ = action_result;
  switch (state_.state_)
  {
    case CommState::WAITING_FOR_GOAL_ACK:
    case CommState::PENDING:
    case CommState::ACTIVE:
    case CommState::WAITING_FOR_RESULT:
    case CommState::WAITING_FOR_CANCEL_ACK:
    case CommState::RECALLING:
    case CommState::PREEMPTING:
    {
      // A little bit of hackery to call all the right state transitions before processing result
      actionlib_msgs::GoalStatusArrayPtr status_array(new actionlib_msgs::GoalStatusArray());
      status_array->status_list.push_back(action_result->status);
      updateStatus(gh, status_array);

      transitionToState(gh, CommState::DONE);
      break;
    }
    case CommState::DONE:
      ROS_ERROR_NAMED("actionlib", "Got a result when we were already in the DONE state"); break;
    default:
      ROS_ERROR_NAMED("actionlib", "In a funny comm state: %u", state_.state_); break;
  }
}

template <class ActionSpec>
void CommStateMachine<ActionSpec>::updateStatus(GoalHandleT& gh, const actionlib_msgs::GoalStatusArrayConstPtr& status_array)
{
  const actionlib_msgs::GoalStatus* goal_status = findGoalStatus(status_array->status_list);

  // It's possible to receive old GoalStatus messages over the wire, even after receiving Result with a terminal state.
  //   Thus, we want to ignore all status that we get after we're done, because it is irrelevant. (See trac #2721)
  if (state_ == CommState::DONE)
    return;

  if (goal_status)
    latest_goal_status_ = *goal_status;
  else
  {
    if (state_ != CommState::WAITING_FOR_GOAL_ACK &&
        state_ != CommState::WAITING_FOR_RESULT &&
        state_ != CommState::DONE)
    {
      processLost(gh);
    }
    return;
  }

  switch (state_.state_)
  {
    case CommState::WAITING_FOR_GOAL_ACK:
    {
      if (goal_status)
      {
        switch (goal_status->status)
        {
          case actionlib_msgs::GoalStatus::PENDING:
            transitionToState(gh, CommState::PENDING);
            break;
          case actionlib_msgs::GoalStatus::ACTIVE:
            transitionToState(gh, CommState::ACTIVE);
            break;
          case actionlib_msgs::GoalStatus::PREEMPTED:
            transitionToState(gh, CommState::ACTIVE);
            transitionToState(gh, CommState::PREEMPTING);
            transitionToState(gh, CommState::WAITING_FOR_RESULT);
            break;
          case actionlib_msgs::GoalStatus::SUCCEEDED:
            transitionToState(gh, CommState::ACTIVE);
            transitionToState(gh, CommState::WAITING_FOR_RESULT);
            break;
          case actionlib_msgs::GoalStatus::ABORTED:
            transitionToState(gh, CommState::ACTIVE);
            transitionToState(gh, CommState::WAITING_FOR_RESULT);
            break;
          case actionlib_msgs::GoalStatus::REJECTED:
            transitionToState(gh, CommState::PENDING);
            transitionToState(gh, CommState::WAITING_FOR_RESULT);
            break;
          case actionlib_msgs::GoalStatus::RECALLED:
            transitionToState(gh, CommState::PENDING);
            transitionToState(gh, CommState::WAITING_FOR_RESULT);
            break;
          case actionlib_msgs::GoalStatus::PREEMPTING:
            transitionToState(gh, CommState::ACTIVE);
            transitionToState(gh, CommState::PREEMPTING);
            break;
          case actionlib_msgs::GoalStatus::RECALLING:
            transitionToState(gh, CommState::PENDING);
            transitionToState(gh, CommState::RECALLING);
            break;
          default:
            ROS_ERROR_NAMED("actionlib", "BUG: Got an unknown status from the ActionServer. status = %u", goal_status->status);
            break;
        }
      }
      break;
    }
    case CommState::PENDING:
    {
      switch (goal_status->status)
      {
        case actionlib_msgs::GoalStatus::PENDING:
          break;
        case actionlib_msgs::GoalStatus::ACTIVE:
          transitionToState(gh, CommState::ACTIVE);
          break;
        case actionlib_msgs::GoalStatus::PREEMPTED:
          transitionToState(gh, CommState::ACTIVE);
          transitionToState(gh, CommState::PREEMPTING);
          transitionToState(gh, CommState::WAITING_FOR_RESULT);
          break;
        case actionlib_msgs::GoalStatus::SUCCEEDED:
          transitionToState(gh, CommState::ACTIVE);
          transitionToState(gh, CommState::WAITING_FOR_RESULT);
          break;
        case actionlib_msgs::GoalStatus::ABORTED:
          transitionToState(gh, CommState::ACTIVE);
          transitionToState(gh, CommState::WAITING_FOR_RESULT);
          break;
        case actionlib_msgs::GoalStatus::REJECTED:
          transitionToState(gh, CommState::WAITING_FOR_RESULT);
          break;
        case actionlib_msgs::GoalStatus::RECALLED:
          transitionToState(gh, CommState::RECALLING);
          transitionToState(gh, CommState::WAITING_FOR_RESULT);
          break;
        case actionlib_msgs::GoalStatus::PREEMPTING:
          transitionToState(gh, CommState::ACTIVE);
          transitionToState(gh, CommState::PREEMPTING);
          break;
        case actionlib_msgs::GoalStatus::RECALLING:
          transitionToState(gh, CommState::RECALLING);
          break;
        default:
          ROS_ERROR_NAMED("actionlib", "BUG: Got an unknown goal status from the ActionServer. status = %u", goal_status->status);
          break;
      }
      break;
    }
    case CommState::ACTIVE:
    {
      switch (goal_status->status)
      {
        case actionlib_msgs::GoalStatus::PENDING:
          ROS_ERROR_NAMED("actionlib", "Invalid transition from ACTIVE to PENDING"); break;
        case actionlib_msgs::GoalStatus::ACTIVE:
          break;
        case actionlib_msgs::GoalStatus::REJECTED:
          ROS_ERROR_NAMED("actionlib", "Invalid transition from ACTIVE to REJECTED"); break;
        case actionlib_msgs::GoalStatus::RECALLING:
          ROS_ERROR_NAMED("actionlib", "Invalid transition from ACTIVE to RECALLING"); break;
        case actionlib_msgs::GoalStatus::RECALLED:
          ROS_ERROR_NAMED("actionlib", "Invalid transition from ACTIVE to RECALLED"); break;
        case actionlib_msgs::GoalStatus::PREEMPTED:
          transitionToState(gh, CommState::PREEMPTING);
          transitionToState(gh, CommState::WAITING_FOR_RESULT);
          break;
        case actionlib_msgs::GoalStatus::SUCCEEDED:
        case actionlib_msgs::GoalStatus::ABORTED:
          transitionToState(gh, CommState::WAITING_FOR_RESULT); break;
        case actionlib_msgs::GoalStatus::PREEMPTING:
          transitionToState(gh, CommState::PREEMPTING); break;
        default:
          ROS_ERROR_NAMED("actionlib", "BUG: Got an unknown goal status from the ActionServer. status = %u", goal_status->status);
          break;
      }
      break;
    }
    case CommState::WAITING_FOR_RESULT:
    {
      switch (goal_status->status)
      {
        case actionlib_msgs::GoalStatus::PENDING :
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from WAITING_FOR_RESUT to PENDING"); break;
        case actionlib_msgs::GoalStatus::PREEMPTING:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from WAITING_FOR_RESUT to PREEMPTING"); break;
        case actionlib_msgs::GoalStatus::RECALLING:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from WAITING_FOR_RESUT to RECALLING"); break;
        case actionlib_msgs::GoalStatus::ACTIVE:
        case actionlib_msgs::GoalStatus::PREEMPTED:
        case actionlib_msgs::GoalStatus::SUCCEEDED:
        case actionlib_msgs::GoalStatus::ABORTED:
        case actionlib_msgs::GoalStatus::REJECTED:
        case actionlib_msgs::GoalStatus::RECALLED:
          break;
        default:
          ROS_ERROR_NAMED("actionlib", "BUG: Got an unknown state from the ActionServer. status = %u", goal_status->status);
          break;
      }
      break;
    }
    case CommState::WAITING_FOR_CANCEL_ACK:
    {
      switch (goal_status->status)
      {
        case actionlib_msgs::GoalStatus::PENDING:
          break;
        case actionlib_msgs::GoalStatus::ACTIVE:
          break;
        case actionlib_msgs::GoalStatus::SUCCEEDED:
        case actionlib_msgs::GoalStatus::ABORTED:
        case actionlib_msgs::GoalStatus::PREEMPTED:
          transitionToState(gh, CommState::PREEMPTING);
          transitionToState(gh, CommState::WAITING_FOR_RESULT);
          break;
        case actionlib_msgs::GoalStatus::RECALLED:
          transitionToState(gh, CommState::RECALLING);
          transitionToState(gh, CommState::WAITING_FOR_RESULT);
          break;
        case actionlib_msgs::GoalStatus::REJECTED:
          transitionToState(gh, CommState::WAITING_FOR_RESULT); break;
        case actionlib_msgs::GoalStatus::PREEMPTING:
          transitionToState(gh, CommState::PREEMPTING); break;
        case actionlib_msgs::GoalStatus::RECALLING:
          transitionToState(gh, CommState::RECALLING); break;
        default:
          ROS_ERROR_NAMED("actionlib", "BUG: Got an unknown state from the ActionServer. status = %u", goal_status->status);
          break;
      }
      break;
    }
    case CommState::RECALLING:
    {
      switch (goal_status->status)
      {
        case actionlib_msgs::GoalStatus::PENDING:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from RECALLING to PENDING"); break;
        case actionlib_msgs::GoalStatus::ACTIVE:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from RECALLING to ACTIVE"); break;
        case actionlib_msgs::GoalStatus::SUCCEEDED:
        case actionlib_msgs::GoalStatus::ABORTED:
        case actionlib_msgs::GoalStatus::PREEMPTED:
          transitionToState(gh, CommState::PREEMPTING);
          transitionToState(gh, CommState::WAITING_FOR_RESULT);
          break;
        case actionlib_msgs::GoalStatus::RECALLED:
          transitionToState(gh, CommState::WAITING_FOR_RESULT);
          break;
        case actionlib_msgs::GoalStatus::REJECTED:
          transitionToState(gh, CommState::WAITING_FOR_RESULT); break;
        case actionlib_msgs::GoalStatus::PREEMPTING:
          transitionToState(gh, CommState::PREEMPTING); break;
        case actionlib_msgs::GoalStatus::RECALLING:
          break;
        default:
          ROS_ERROR_NAMED("actionlib", "BUG: Got an unknown state from the ActionServer. status = %u", goal_status->status);
          break;
      }
      break;
    }
    case CommState::PREEMPTING:
    {
      switch (goal_status->status)
      {
        case actionlib_msgs::GoalStatus::PENDING:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from PREEMPTING to PENDING"); break;
        case actionlib_msgs::GoalStatus::ACTIVE:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from PREEMPTING to ACTIVE"); break;
        case actionlib_msgs::GoalStatus::REJECTED:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from PREEMPTING to REJECTED"); break;
        case actionlib_msgs::GoalStatus::RECALLING:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from PREEMPTING to RECALLING"); break;
        case actionlib_msgs::GoalStatus::RECALLED:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from PREEMPTING to RECALLED"); break;
          break;
        case actionlib_msgs::GoalStatus::PREEMPTED:
        case actionlib_msgs::GoalStatus::SUCCEEDED:
        case actionlib_msgs::GoalStatus::ABORTED:
          transitionToState(gh, CommState::WAITING_FOR_RESULT); break;
        case actionlib_msgs::GoalStatus::PREEMPTING:
          break;
        default:
          ROS_ERROR_NAMED("actionlib", "BUG: Got an unknown state from the ActionServer. status = %u", goal_status->status);
          break;
      }
      break;
    }
    case CommState::DONE:
    {
      switch (goal_status->status)
      {
        case actionlib_msgs::GoalStatus::PENDING:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from DONE to PENDING"); break;
        case actionlib_msgs::GoalStatus::ACTIVE:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from DONE to ACTIVE"); break;
        case actionlib_msgs::GoalStatus::RECALLING:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from DONE to RECALLING"); break;
        case actionlib_msgs::GoalStatus::PREEMPTING:
          ROS_ERROR_NAMED("actionlib", "Invalid Transition from DONE to PREEMPTING"); break;
        case actionlib_msgs::GoalStatus::PREEMPTED:
        case actionlib_msgs::GoalStatus::SUCCEEDED:
        case actionlib_msgs::GoalStatus::ABORTED:
        case actionlib_msgs::GoalStatus::RECALLED:
        case actionlib_msgs::GoalStatus::REJECTED:
          break;
        default:
          ROS_ERROR_NAMED("actionlib", "BUG: Got an unknown state from the ActionServer. status = %u", goal_status->status);
          break;
      }
      break;
    }
    default:
      ROS_ERROR_NAMED("actionlib", "In a funny comm state: %u", state_.state_);
      break;
  }
}


template <class ActionSpec>
void CommStateMachine<ActionSpec>::processLost(GoalHandleT& gh)
{
  ROS_WARN_NAMED("actionlib", "Transitioning goal to LOST");
  latest_goal_status_.status = actionlib_msgs::GoalStatus::LOST;
  transitionToState(gh, CommState::DONE);
}

template <class ActionSpec>
void CommStateMachine<ActionSpec>::transitionToState(GoalHandleT& gh, const CommState::StateEnum& next_state)
{
  transitionToState(gh, CommState(next_state));
}

template <class ActionSpec>
void CommStateMachine<ActionSpec>::transitionToState(GoalHandleT& gh, const CommState& next_state)
{
  ROS_DEBUG_NAMED("actionlib", "Trying to transition to %s", next_state.toString().c_str());
  setCommState(next_state);
  if (transition_cb_)
    transition_cb_(gh);
}

}
