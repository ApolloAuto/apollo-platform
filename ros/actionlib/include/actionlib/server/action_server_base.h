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
#ifndef ACTION_LIB_ACTION_SERVER_BASE
#define ACTION_LIB_ACTION_SERVER_BASE

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib/enclosure_deleter.h>
#include <actionlib/goal_id_generator.h>
#include <actionlib/action_definition.h>
#include <actionlib/server/status_tracker.h>
#include <actionlib/server/handle_tracker_deleter.h>
#include <actionlib/server/server_goal_handle.h>
#include <actionlib/destruction_guard.h>

#include <list>

namespace actionlib {
  /**
   * @class ActionServerBase
   * @brief The ActionServerBase implements the logic for an ActionServer.
   */
  template <class ActionSpec>
  class ActionServerBase {
    public:
      //for convenience when referring to ServerGoalHandles
      typedef ServerGoalHandle<ActionSpec> GoalHandle;

      //generates typedefs that we'll use to make our lives easier
      ACTION_DEFINITION(ActionSpec);

      /**
       * @brief  Constructor for an ActionServer
       * @param  goal_cb A goal callback to be called when the ActionServer receives a new goal over the wire
       * @param  cancel_cb A cancel callback to be called when the ActionServer receives a new cancel request over the wire
       * @param  auto_start A boolean value that tells the ActionServer wheteher or not to start publishing as soon as it comes up. THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS and start() should be called after construction of the server.
       */
      ActionServerBase(
          boost::function<void (GoalHandle)> goal_cb,
          boost::function<void (GoalHandle)> cancel_cb,
          bool auto_start = false);


      /**
       * @brief  Destructor for the ActionServerBase
       */
      virtual ~ActionServerBase();

      /**
       * @brief  Register a callback to be invoked when a new goal is received, this will replace any  previously registered callback
       * @param  cb The callback to invoke
       */
      void registerGoalCallback(boost::function<void (GoalHandle)> cb);

      /**
       * @brief  Register a callback to be invoked when a new cancel is received, this will replace any  previously registered callback
       * @param  cb The callback to invoke
       */
      void registerCancelCallback(boost::function<void (GoalHandle)> cb);

      /**
       * @brief  Explicitly start the action server, used it auto_start is set to false
       */
      void start();


      /**
       * @brief  The ROS callback for goals coming into the ActionServerBase
       */
      void goalCallback(const boost::shared_ptr<const ActionGoal>& goal);

      /**
       * @brief  The ROS callback for cancel requests coming into the ActionServerBase
       */
      void cancelCallback(const boost::shared_ptr<const actionlib_msgs::GoalID>& goal_id);

    protected:
      // Allow access to protected fields for helper classes
      friend class ServerGoalHandle<ActionSpec>;
      friend class HandleTrackerDeleter<ActionSpec>;

      /**
       * @brief  Initialize all ROS connections and setup timers
       */
      virtual void initialize() = 0;

      /**
       * @brief  Publishes a result for a given goal
       * @param status The status of the goal with which the result is associated
       * @param result The result to publish
       */
      virtual void publishResult(const actionlib_msgs::GoalStatus& status, const Result& result) = 0;

      /**
       * @brief  Publishes feedback for a given goal
       * @param status The status of the goal with which the feedback is associated
       * @param feedback The feedback to publish
       */
      virtual void publishFeedback(const actionlib_msgs::GoalStatus& status, const Feedback& feedback) = 0;

      /**
       * @brief  Explicitly publish status
       */
      virtual void publishStatus() = 0;

      boost::recursive_mutex lock_;

      std::list<StatusTracker<ActionSpec> > status_list_;

      boost::function<void (GoalHandle)> goal_callback_;
      boost::function<void (GoalHandle)> cancel_callback_;

      ros::Time last_cancel_;
      ros::Duration status_list_timeout_;

      GoalIDGenerator id_generator_;
      bool started_;
      boost::shared_ptr<DestructionGuard> guard_;
  };

  template <class ActionSpec>
  ActionServerBase<ActionSpec>::ActionServerBase(
      boost::function<void (GoalHandle)> goal_cb,
      boost::function<void (GoalHandle)> cancel_cb,
      bool auto_start) :
    goal_callback_(goal_cb),
    cancel_callback_(cancel_cb),
    started_(auto_start),
    guard_(new DestructionGuard)
  {
  }

  template <class ActionSpec>
  ActionServerBase<ActionSpec>::~ActionServerBase()
  {
    // Block until we can safely destruct
    guard_->destruct();
  }

  template <class ActionSpec>
  void ActionServerBase<ActionSpec>::registerGoalCallback(boost::function<void (GoalHandle)> cb) 
  {
    goal_callback_ = cb;
  }

  template <class ActionSpec>
  void ActionServerBase<ActionSpec>::registerCancelCallback(boost::function<void (GoalHandle)> cb) 
  {
    cancel_callback_ = cb;
  }

  template <class ActionSpec>
  void ActionServerBase<ActionSpec>::start()
  {
    initialize();
    started_ = true;
    publishStatus();
  }


  template <class ActionSpec>
  void ActionServerBase<ActionSpec>::goalCallback(const boost::shared_ptr<const ActionGoal>& goal)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);

    //if we're not started... then we're not actually going to do anything
    if(!started_)
      return;

    ROS_DEBUG_NAMED("actionlib", "The action server has received a new goal request");

    //we need to check if this goal already lives in the status list
    for(typename std::list<StatusTracker<ActionSpec> >::iterator it = status_list_.begin(); it != status_list_.end(); ++it){
      if(goal->goal_id.id == (*it).status_.goal_id.id){

        // The goal could already be in a recalling state if a cancel came in before the goal
        if ( (*it).status_.status == actionlib_msgs::GoalStatus::RECALLING ) {
          (*it).status_.status = actionlib_msgs::GoalStatus::RECALLED;
          publishResult((*it).status_, Result());
        }

        //if this is a request for a goal that has no active handles left,
        //we'll bump how long it stays in the list
        if((*it).handle_tracker_.expired()){
          (*it).handle_destruction_time_ = goal->goal_id.stamp;
        }

        //make sure not to call any user callbacks or add duplicate status onto the list
        return;
      }
    }

    //if the goal is not in our list, we need to create a StatusTracker associated with this goal and push it on
    typename std::list<StatusTracker<ActionSpec> >::iterator it = status_list_.insert(status_list_.end(), StatusTracker<ActionSpec> (goal));

    //we need to create a handle tracker for the incoming goal and update the StatusTracker
    HandleTrackerDeleter<ActionSpec> d(this, it, guard_);
    boost::shared_ptr<void> handle_tracker((void *)NULL, d);
    (*it).handle_tracker_ = handle_tracker;

    //check if this goal has already been canceled based on its timestamp
    if(goal->goal_id.stamp != ros::Time() && goal->goal_id.stamp <= last_cancel_){
      //if it has... just create a GoalHandle for it and setCanceled
      GoalHandle gh(it, this, handle_tracker, guard_);
      gh.setCanceled(Result(), "This goal handle was canceled by the action server because its timestamp is before the timestamp of the last cancel request");
    }
    else{
      GoalHandle gh = GoalHandle(it, this, handle_tracker, guard_);

      //make sure that we unlock before calling the users callback
      lock_.unlock();

      //now, we need to create a goal handle and call the user's callback
      goal_callback_(gh);

      lock_.lock();
    }
  }

  template <class ActionSpec>
  void ActionServerBase<ActionSpec>::cancelCallback(const boost::shared_ptr<const actionlib_msgs::GoalID>& goal_id)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);

    //if we're not started... then we're not actually going to do anything
    if(!started_)
      return;

    //we need to handle a cancel for the user
    ROS_DEBUG_NAMED("actionlib", "The action server has received a new cancel request");
    bool goal_id_found = false;
    for(typename std::list<StatusTracker<ActionSpec> >::iterator it = status_list_.begin(); it != status_list_.end(); ++it){
      //check if the goal id is zero or if it is equal to the goal id of
      //the iterator or if the time of the iterator warrants a cancel
      if(
          (goal_id->id == "" && goal_id->stamp == ros::Time()) //id and stamp 0 --> cancel everything
          || goal_id->id == (*it).status_.goal_id.id //ids match... cancel that goal
          || (goal_id->stamp != ros::Time() && (*it).status_.goal_id.stamp <= goal_id->stamp) //stamp != 0 --> cancel everything before stamp
        ){
        //we need to check if we need to store this cancel request for later
        if(goal_id->id == (*it).status_.goal_id.id)
          goal_id_found = true;

        //attempt to get the handle_tracker for the list item if it exists
        boost::shared_ptr<void> handle_tracker = (*it).handle_tracker_.lock();

        if((*it).handle_tracker_.expired()){
          //if the handle tracker is expired, then we need to create a new one
          HandleTrackerDeleter<ActionSpec> d(this, it, guard_);
          handle_tracker = boost::shared_ptr<void>((void *)NULL, d);
          (*it).handle_tracker_ = handle_tracker;

          //we also need to reset the time that the status is supposed to be removed from the list
          (*it).handle_destruction_time_ = ros::Time();
        }

        //set the status of the goal to PREEMPTING or RECALLING as approriate
        //and check if the request should be passed on to the user
        GoalHandle gh(it, this, handle_tracker, guard_);
        if(gh.setCancelRequested()){
          //make sure that we're unlocked before we call the users callback
          lock_.unlock();

          //call the user's cancel callback on the relevant goal
          cancel_callback_(gh);

          //lock for further modification of the status list
          lock_.lock();
        }
      }
    }

    //if the requested goal_id was not found, and it is non-zero, then we need to store the cancel request
    if(goal_id->id != "" && !goal_id_found){
      typename std::list<StatusTracker<ActionSpec> >::iterator it = status_list_.insert(status_list_.end(),
          StatusTracker<ActionSpec> (*goal_id, actionlib_msgs::GoalStatus::RECALLING));
      //start the timer for how long the status will live in the list without a goal handle to it
      (*it).handle_destruction_time_ = goal_id->stamp;
    }

    //make sure to set last_cancel_ based on the stamp associated with this cancel request
    if(goal_id->stamp > last_cancel_)
      last_cancel_ = goal_id->stamp;
  }
}


#endif
