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
#ifndef ACTION_DEFINITION_H_
#define ACTION_DEFINITION_H_
//A macro that will generate helpful typedefs for action client, server, and policy implementers
namespace actionlib {
#define ACTION_DEFINITION(ActionSpec) \
  typedef typename ActionSpec::_action_goal_type ActionGoal; \
  typedef typename ActionGoal::_goal_type Goal; \
  typedef typename ActionSpec::_action_result_type ActionResult; \
  typedef typename ActionResult::_result_type Result; \
  typedef typename ActionSpec::_action_feedback_type ActionFeedback; \
  typedef typename ActionFeedback::_feedback_type Feedback; \
  \
  typedef boost::shared_ptr<const ActionGoal> ActionGoalConstPtr; \
  typedef boost::shared_ptr<ActionGoal> ActionGoalPtr; \
  typedef boost::shared_ptr<const Goal> GoalConstPtr;\
  \
  typedef boost::shared_ptr<const ActionResult> ActionResultConstPtr; \
  typedef boost::shared_ptr<const Result> ResultConstPtr;\
  \
  typedef boost::shared_ptr<const ActionFeedback> ActionFeedbackConstPtr; \
  typedef boost::shared_ptr<const Feedback> FeedbackConstPtr;
};
#endif

