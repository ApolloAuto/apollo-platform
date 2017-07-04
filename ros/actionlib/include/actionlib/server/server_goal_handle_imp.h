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
#ifndef ACTIONLIB_SERVER_GOAL_HANDLE_IMP_H_
#define ACTIONLIB_SERVER_GOAL_HANDLE_IMP_H_
namespace actionlib {
  template <class ActionSpec>
  ServerGoalHandle<ActionSpec>::ServerGoalHandle() : as_(NULL) {}

  template <class ActionSpec>
  ServerGoalHandle<ActionSpec>::ServerGoalHandle(const ServerGoalHandle& gh): 
    status_it_(gh.status_it_), goal_(gh.goal_), as_(gh.as_), handle_tracker_(gh.handle_tracker_), guard_(gh.guard_){}

  template <class ActionSpec>
  void ServerGoalHandle<ActionSpec>::setAccepted(const std::string& text){
    if(as_ == NULL){
      ROS_ERROR_NAMED("actionlib", "You are attempting to call methods on an uninitialized goal handle");
      return;
    }

    //check to see if we can use the action server
    DestructionGuard::ScopedProtector protector(*guard_);
    if(!protector.isProtected()){
      ROS_ERROR_NAMED("actionlib", "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
      return;
    }

    ROS_DEBUG_NAMED("actionlib", "Accepting goal, id: %s, stamp: %.2f", getGoalID().id.c_str(), getGoalID().stamp.toSec());
    if(goal_){
      boost::recursive_mutex::scoped_lock lock(as_->lock_);
      unsigned int status = (*status_it_).status_.status;

      //if we were pending before, then we'll go active
      if(status == actionlib_msgs::GoalStatus::PENDING){
        (*status_it_).status_.status = actionlib_msgs::GoalStatus::ACTIVE;
        (*status_it_).status_.text = text;
        as_->publishStatus();
      }
      //if we were recalling before, now we'll go to preempting
      else if(status == actionlib_msgs::GoalStatus::RECALLING){
        (*status_it_).status_.status = actionlib_msgs::GoalStatus::PREEMPTING;
        (*status_it_).status_.text = text;
        as_->publishStatus();
      }
      else
        ROS_ERROR_NAMED("actionlib", "To transition to an active state, the goal must be in a pending or recalling state, it is currently in state: %d",
            (*status_it_).status_.status);
    }
    else
      ROS_ERROR_NAMED("actionlib", "Attempt to set status on an uninitialized ServerGoalHandle");
  }

  template <class ActionSpec>
  void ServerGoalHandle<ActionSpec>::setCanceled(const Result& result, const std::string& text){
    if(as_ == NULL){
      ROS_ERROR_NAMED("actionlib", "You are attempting to call methods on an uninitialized goal handle");
      return;
    }

    //check to see if we can use the action server
    DestructionGuard::ScopedProtector protector(*guard_);
    if(!protector.isProtected()){
      ROS_ERROR_NAMED("actionlib", "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
      return;
    }

    ROS_DEBUG_NAMED("actionlib", "Setting status to canceled on goal, id: %s, stamp: %.2f", getGoalID().id.c_str(), getGoalID().stamp.toSec());
    if(goal_){
      boost::recursive_mutex::scoped_lock lock(as_->lock_);
      unsigned int status = (*status_it_).status_.status;
      if(status == actionlib_msgs::GoalStatus::PENDING || status == actionlib_msgs::GoalStatus::RECALLING){
        (*status_it_).status_.status = actionlib_msgs::GoalStatus::RECALLED;
        (*status_it_).status_.text = text;
        as_->publishResult((*status_it_).status_, result);
      }
      else if(status == actionlib_msgs::GoalStatus::ACTIVE || status == actionlib_msgs::GoalStatus::PREEMPTING){
        (*status_it_).status_.status = actionlib_msgs::GoalStatus::PREEMPTED;
        (*status_it_).status_.text = text;
        as_->publishResult((*status_it_).status_, result);
      }
      else
        ROS_ERROR_NAMED("actionlib", "To transition to a cancelled state, the goal must be in a pending, recalling, active, or preempting state, it is currently in state: %d",
            (*status_it_).status_.status);
    }
    else
      ROS_ERROR_NAMED("actionlib", "Attempt to set status on an uninitialized ServerGoalHandle");
  }

  template <class ActionSpec>
  void ServerGoalHandle<ActionSpec>::setRejected(const Result& result, const std::string& text){
    if(as_ == NULL){
      ROS_ERROR_NAMED("actionlib", "You are attempting to call methods on an uninitialized goal handle");
      return;
    }

    //check to see if we can use the action server
    DestructionGuard::ScopedProtector protector(*guard_);
    if(!protector.isProtected()){
      ROS_ERROR_NAMED("actionlib", "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
      return;
    }

    ROS_DEBUG_NAMED("actionlib", "Setting status to rejected on goal, id: %s, stamp: %.2f", getGoalID().id.c_str(), getGoalID().stamp.toSec());
    if(goal_){
      boost::recursive_mutex::scoped_lock lock(as_->lock_);
      unsigned int status = (*status_it_).status_.status;
      if(status == actionlib_msgs::GoalStatus::PENDING || status == actionlib_msgs::GoalStatus::RECALLING){
        (*status_it_).status_.status = actionlib_msgs::GoalStatus::REJECTED;
        (*status_it_).status_.text = text;
        as_->publishResult((*status_it_).status_, result);
      }
      else
        ROS_ERROR_NAMED("actionlib", "To transition to a rejected state, the goal must be in a pending or recalling state, it is currently in state: %d",
            (*status_it_).status_.status);
    }
    else
      ROS_ERROR_NAMED("actionlib", "Attempt to set status on an uninitialized ServerGoalHandle");
  }

  template <class ActionSpec>
  void ServerGoalHandle<ActionSpec>::setAborted(const Result& result, const std::string& text){
    if(as_ == NULL){
      ROS_ERROR_NAMED("actionlib", "You are attempting to call methods on an uninitialized goal handle");
      return;
    }

    //check to see if we can use the action server
    DestructionGuard::ScopedProtector protector(*guard_);
    if(!protector.isProtected()){
      ROS_ERROR_NAMED("actionlib", "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
      return;
    }

    ROS_DEBUG_NAMED("actionlib", "Setting status to aborted on goal, id: %s, stamp: %.2f", getGoalID().id.c_str(), getGoalID().stamp.toSec());
    if(goal_){
      boost::recursive_mutex::scoped_lock lock(as_->lock_);
      unsigned int status = (*status_it_).status_.status;
      if(status == actionlib_msgs::GoalStatus::PREEMPTING || status == actionlib_msgs::GoalStatus::ACTIVE){
        (*status_it_).status_.status = actionlib_msgs::GoalStatus::ABORTED;
        (*status_it_).status_.text = text;
        as_->publishResult((*status_it_).status_, result);
      }
      else
        ROS_ERROR_NAMED("actionlib", "To transition to an aborted state, the goal must be in a preempting or active state, it is currently in state: %d",
            status);
    }
    else
      ROS_ERROR_NAMED("actionlib", "Attempt to set status on an uninitialized ServerGoalHandle");
  }

  template <class ActionSpec>
  void ServerGoalHandle<ActionSpec>::setSucceeded(const Result& result, const std::string& text){
    if(as_ == NULL){
      ROS_ERROR_NAMED("actionlib", "You are attempting to call methods on an uninitialized goal handle");
      return;
    }

    //check to see if we can use the action server
    DestructionGuard::ScopedProtector protector(*guard_);
    if(!protector.isProtected()){
      ROS_ERROR_NAMED("actionlib", "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
      return;
    }

    ROS_DEBUG_NAMED("actionlib", "Setting status to succeeded on goal, id: %s, stamp: %.2f", getGoalID().id.c_str(), getGoalID().stamp.toSec());
    if(goal_){
      boost::recursive_mutex::scoped_lock lock(as_->lock_);
      unsigned int status = (*status_it_).status_.status;
      if(status == actionlib_msgs::GoalStatus::PREEMPTING || status == actionlib_msgs::GoalStatus::ACTIVE){
        (*status_it_).status_.status = actionlib_msgs::GoalStatus::SUCCEEDED;
        (*status_it_).status_.text = text;
        as_->publishResult((*status_it_).status_, result);
      }
      else
        ROS_ERROR_NAMED("actionlib", "To transition to a succeeded state, the goal must be in a preempting or active state, it is currently in state: %d",
            status);
    }
    else
      ROS_ERROR_NAMED("actionlib", "Attempt to set status on an uninitialized ServerGoalHandle");
  }

  template <class ActionSpec>
  void ServerGoalHandle<ActionSpec>::publishFeedback(const Feedback& feedback){
    if(as_ == NULL){
      ROS_ERROR_NAMED("actionlib", "You are attempting to call methods on an uninitialized goal handle");
      return;
    }

    //check to see if we can use the action server
    DestructionGuard::ScopedProtector protector(*guard_);
    if(!protector.isProtected()){
      ROS_ERROR_NAMED("actionlib", "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
      return;
    }

    ROS_DEBUG_NAMED("actionlib", "Publishing feedback for goal, id: %s, stamp: %.2f", getGoalID().id.c_str(), getGoalID().stamp.toSec());
    if(goal_) {
      boost::recursive_mutex::scoped_lock lock(as_->lock_);
      as_->publishFeedback((*status_it_).status_, feedback);
    }
    else
      ROS_ERROR_NAMED("actionlib", "Attempt to publish feedback on an uninitialized ServerGoalHandle");
  }

  template <class ActionSpec>
  bool ServerGoalHandle<ActionSpec>::isValid() const{
    return goal_ && as_!= NULL;
  }

  template <class ActionSpec>
  boost::shared_ptr<const typename ServerGoalHandle<ActionSpec>::Goal> ServerGoalHandle<ActionSpec>::getGoal() const{
    //if we have a goal that is non-null
    if(goal_){
      //create the deleter for our goal subtype
      EnclosureDeleter<const ActionGoal> d(goal_);
      return boost::shared_ptr<const Goal>(&(goal_->goal), d);
    }
    return boost::shared_ptr<const Goal>();
  }

  template <class ActionSpec>
  actionlib_msgs::GoalID ServerGoalHandle<ActionSpec>::getGoalID() const{
    if(goal_ && as_!= NULL){
      DestructionGuard::ScopedProtector protector(*guard_);
      if(protector.isProtected()){
        boost::recursive_mutex::scoped_lock lock(as_->lock_);
        return (*status_it_).status_.goal_id;
      }
      else
        return actionlib_msgs::GoalID();
    }
    else{
      ROS_ERROR_NAMED("actionlib", "Attempt to get a goal id on an uninitialized ServerGoalHandle or one that has no ActionServer associated with it.");
      return actionlib_msgs::GoalID();
    }
  }

  template <class ActionSpec>
  actionlib_msgs::GoalStatus ServerGoalHandle<ActionSpec>::getGoalStatus() const{
    if(goal_ && as_!= NULL){
      DestructionGuard::ScopedProtector protector(*guard_);
      if(protector.isProtected()){
        boost::recursive_mutex::scoped_lock lock(as_->lock_);
        return (*status_it_).status_;
      }
      else
        return actionlib_msgs::GoalStatus();
    }
    else{
      ROS_ERROR_NAMED("actionlib", "Attempt to get goal status on an uninitialized ServerGoalHandle or one that has no ActionServer associated with it.");
      return actionlib_msgs::GoalStatus();
    }
  }

  template <class ActionSpec>
  ServerGoalHandle<ActionSpec>& ServerGoalHandle<ActionSpec>::operator=(const ServerGoalHandle& gh){
    status_it_ = gh.status_it_;
    goal_ = gh.goal_;
    as_ = gh.as_;
    handle_tracker_ = gh.handle_tracker_;
    guard_ = gh.guard_;
    return *this;
  }

  template <class ActionSpec>
  bool ServerGoalHandle<ActionSpec>::operator==(const ServerGoalHandle& other) const{
    if(!goal_ && !other.goal_)
      return true;

    if(!goal_ || !other.goal_)
      return false;

    actionlib_msgs::GoalID my_id = getGoalID();
    actionlib_msgs::GoalID their_id = other.getGoalID();
    return my_id.id == their_id.id;
  }

  template <class ActionSpec>
  bool ServerGoalHandle<ActionSpec>::operator!=(const ServerGoalHandle& other) const{
    return !(*this == other);
  }

  template <class ActionSpec>
  ServerGoalHandle<ActionSpec>::ServerGoalHandle(typename std::list<StatusTracker<ActionSpec> >::iterator status_it,
      ActionServerBase<ActionSpec>* as, boost::shared_ptr<void> handle_tracker, boost::shared_ptr<DestructionGuard> guard)
    : status_it_(status_it), goal_((*status_it).goal_),
    as_(as), handle_tracker_(handle_tracker), guard_(guard){}

  template <class ActionSpec>
  bool ServerGoalHandle<ActionSpec>::setCancelRequested(){
    if(as_ == NULL){
      ROS_ERROR_NAMED("actionlib", "You are attempting to call methods on an uninitialized goal handle");
      return false;
    }

    //check to see if we can use the action server
    DestructionGuard::ScopedProtector protector(*guard_);
    if(!protector.isProtected()){
      ROS_ERROR_NAMED("actionlib", "The ActionServer associated with this GoalHandle is invalid. Did you delete the ActionServer before the GoalHandle?");
      return false;
    }

    ROS_DEBUG_NAMED("actionlib", "Transisitoning to a cancel requested state on goal id: %s, stamp: %.2f", getGoalID().id.c_str(), getGoalID().stamp.toSec());
    if(goal_){
      boost::recursive_mutex::scoped_lock lock(as_->lock_);
      unsigned int status = (*status_it_).status_.status;
      if(status == actionlib_msgs::GoalStatus::PENDING){
        (*status_it_).status_.status = actionlib_msgs::GoalStatus::RECALLING;
        as_->publishStatus();
        return true;
      }

      if(status == actionlib_msgs::GoalStatus::ACTIVE){
        (*status_it_).status_.status = actionlib_msgs::GoalStatus::PREEMPTING;
        as_->publishStatus();
        return true;
      }

    }
    return false;
  }
};
#endif
