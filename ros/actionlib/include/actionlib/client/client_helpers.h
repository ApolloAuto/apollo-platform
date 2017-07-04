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

#ifndef ACTIONLIB_GOAL_MANAGER_H_
#define ACTIONLIB_GOAL_MANAGER_H_

#include <boost/thread/recursive_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>


#include "actionlib/action_definition.h"

#include "actionlib/managed_list.h"
#include "actionlib/enclosure_deleter.h"
#include "actionlib/goal_id_generator.h"

#include "actionlib/client/comm_state.h"
#include "actionlib/client/terminal_state.h"

#include "actionlib/destruction_guard.h"

// msgs
#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatusArray.h"

namespace actionlib
{

template <class ActionSpec>
class ClientGoalHandle;

template <class ActionSpec>
class CommStateMachine;

template <class ActionSpec>
class GoalManager
{
public:
  ACTION_DEFINITION(ActionSpec);
  typedef GoalManager<ActionSpec> GoalManagerT;
  typedef ClientGoalHandle<ActionSpec> GoalHandleT;
  typedef boost::function<void (GoalHandleT) > TransitionCallback;
  typedef boost::function<void (GoalHandleT, const FeedbackConstPtr&) > FeedbackCallback;
  typedef boost::function<void (const ActionGoalConstPtr)> SendGoalFunc;
  typedef boost::function<void (const actionlib_msgs::GoalID&)> CancelFunc;

  GoalManager(const boost::shared_ptr<DestructionGuard>& guard) : guard_(guard) { }

  void registerSendGoalFunc(SendGoalFunc send_goal_func);
  void registerCancelFunc(CancelFunc cancel_func);

  GoalHandleT initGoal( const Goal& goal,
                        TransitionCallback transition_cb = TransitionCallback(),
                        FeedbackCallback feedback_cb = FeedbackCallback() );

  void updateStatuses(const actionlib_msgs::GoalStatusArrayConstPtr& status_array);
  void updateFeedbacks(const ActionFeedbackConstPtr& action_feedback);
  void updateResults(const ActionResultConstPtr& action_result);

  friend class ClientGoalHandle<ActionSpec>;

  // should be private
  typedef ManagedList< boost::shared_ptr<CommStateMachine<ActionSpec> > > ManagedListT;
  ManagedListT list_;
private:
  SendGoalFunc send_goal_func_ ;
  CancelFunc cancel_func_ ;

  boost::shared_ptr<DestructionGuard> guard_;

  boost::recursive_mutex list_mutex_;

  GoalIDGenerator id_generator_;

  void listElemDeleter(typename ManagedListT::iterator it);
};

/**
 * \brief Client side handle to monitor goal progress
 *
 * A ClientGoalHandle is a reference counted object that is used to manipulate and monitor the progress
 * of an already dispatched goal. Once all the goal handles go out of scope (or are reset), an
 * ActionClient stops maintaining state for that goal.
 */
template <class ActionSpec>
class ClientGoalHandle
{
private:
  ACTION_DEFINITION(ActionSpec);

public:
  /**
   * \brief Create an empty goal handle
   *
   * Constructs a goal handle that doesn't track any goal. Calling any method on an empty goal
   * handle other than operator= will trigger an assertion.
   */
  ClientGoalHandle();

  ~ClientGoalHandle();

  /**
   * \brief Stops goal handle from tracking a goal
   *
   * Useful if you want to stop tracking the progress of a goal, but it is inconvenient to force
   * the goal handle to go out of scope. Has pretty much the same semantics as boost::shared_ptr::reset()
   */
  void reset();

  /**
   * \brief Checks if this goal handle is tracking a goal
   *
   * Has pretty much the same semantics as boost::shared_ptr::expired()
   * \return True if this goal handle is not tracking a goal
   */
  inline bool isExpired() const;

  /**
   * \brief Get the state of this goal's communication state machine from interaction with the server
   *
   * Possible States are: WAITING_FOR_GOAL_ACK, PENDING, ACTIVE, WAITING_FOR_RESULT,
   *                      WAITING_FOR_CANCEL_ACK, RECALLING, PREEMPTING, DONE
   * \return The current goal's communication state with the server
   */
  CommState getCommState();

  /**
   * \brief Get the terminal state information for this goal
   *
   * Possible States Are: RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
   * This call only makes sense if CommState==DONE. This will send ROS_WARNs if we're not in DONE
   * \return The terminal state
   */
  TerminalState getTerminalState();

  /**
   * \brief Get result associated with this goal
   *
   * \return NULL if no reseult received.  Otherwise returns shared_ptr to result.
   */
  ResultConstPtr getResult();

  /**
   * \brief Resends this goal [with the same GoalID] to the ActionServer
   *
   * Useful if the user thinks that the goal may have gotten lost in transit
   */
  void resend();

  /**
   * \brief Sends a cancel message for this specific goal to the ActionServer
   *
   * Also transitions the Communication State Machine to WAITING_FOR_CANCEL_ACK
   */
  void cancel();

  /**
   * \brief Check if two goal handles point to the same goal
   * \return TRUE if both point to the same goal. Also returns TRUE if both handles are inactive.
   */
  bool operator==(const ClientGoalHandle<ActionSpec>& rhs) const;

  /**
   * \brief !(operator==())
   */
  bool operator!=(const ClientGoalHandle<ActionSpec>& rhs) const;

  friend class GoalManager<ActionSpec>;
private:
  typedef GoalManager<ActionSpec> GoalManagerT;
  typedef ManagedList< boost::shared_ptr<CommStateMachine<ActionSpec> > > ManagedListT;

  ClientGoalHandle(GoalManagerT* gm, typename ManagedListT::Handle handle, const boost::shared_ptr<DestructionGuard>& guard);

  GoalManagerT* gm_;
  bool active_;
  //typename ManagedListT::iterator it_;
  boost::shared_ptr<DestructionGuard> guard_;   // Guard must still exist when the list_handle_ is destroyed
  typename ManagedListT::Handle list_handle_;
};

template <class ActionSpec>
class CommStateMachine
{
  private:
    //generates typedefs that we'll use to make our lives easier
    ACTION_DEFINITION(ActionSpec);

  public:
    typedef boost::function<void (const ClientGoalHandle<ActionSpec>&) > TransitionCallback;
    typedef boost::function<void (const ClientGoalHandle<ActionSpec>&, const FeedbackConstPtr&) > FeedbackCallback;
    typedef ClientGoalHandle<ActionSpec> GoalHandleT;

    CommStateMachine(const ActionGoalConstPtr& action_goal,
                     TransitionCallback transition_callback,
                     FeedbackCallback feedback_callback);

    ActionGoalConstPtr getActionGoal() const;
    CommState getCommState() const;
    actionlib_msgs::GoalStatus getGoalStatus() const;
    ResultConstPtr getResult() const;

    // Transitions caused by messages
    void updateStatus(GoalHandleT& gh, const actionlib_msgs::GoalStatusArrayConstPtr& status_array);
    void updateFeedback(GoalHandleT& gh, const ActionFeedbackConstPtr& feedback);
    void updateResult(GoalHandleT& gh, const ActionResultConstPtr& result);

    // Forced transitions
    void transitionToState(GoalHandleT& gh, const CommState::StateEnum& next_state);
    void transitionToState(GoalHandleT& gh, const CommState& next_state);
    void processLost(GoalHandleT& gh);
  private:
    CommStateMachine();

    // State
    CommState state_;
    ActionGoalConstPtr action_goal_;
    actionlib_msgs::GoalStatus latest_goal_status_;
    ActionResultConstPtr latest_result_;

    // Callbacks
    TransitionCallback transition_cb_;
    FeedbackCallback   feedback_cb_;

    // **** Implementation ****
    //! Change the state, as well as print out ROS_DEBUG info
    void setCommState(const CommState& state);
    void setCommState(const CommState::StateEnum& state);
    const actionlib_msgs::GoalStatus* findGoalStatus(const std::vector<actionlib_msgs::GoalStatus>& status_vec) const;
};

}

#include "actionlib/client/goal_manager_imp.h"
#include "actionlib/client/client_goal_handle_imp.h"
#include "actionlib/client/comm_state_machine_imp.h"

#endif // ACTIONLIB_GOAL_MANAGER_H_
