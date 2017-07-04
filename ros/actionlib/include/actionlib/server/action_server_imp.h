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
#ifndef ACTIONLIB_ACTION_SERVER_IMP_H_
#define ACTIONLIB_ACTION_SERVER_IMP_H_
namespace actionlib {
  template <class ActionSpec>
  ActionServer<ActionSpec>::ActionServer(
      ros::NodeHandle n,
      std::string name,
      bool auto_start) :
    ActionServerBase<ActionSpec>(boost::function<void (GoalHandle)>(),boost::function<void (GoalHandle)>(), auto_start),
    node_(n, name)
  {
    //if we're to autostart... then we'll initialize things
    if(this->started_){
      ROS_WARN_NAMED("actionlib", "You've passed in true for auto_start for the C++ action server at [%s]. You should always pass in false to avoid race conditions.", node_.getNamespace().c_str());
    }
  }

  template <class ActionSpec>
  ActionServer<ActionSpec>::ActionServer(ros::NodeHandle n, std::string name) :
    ActionServerBase<ActionSpec>(boost::function<void (GoalHandle)>(),boost::function<void (GoalHandle)>(), true),
    node_(n, name)
  {
    //if we're to autostart... then we'll initialize things
    if(this->started_){
      ROS_WARN_NAMED("actionlib", "You've passed in true for auto_start for the C++ action server at [%s]. You should always pass in false to avoid race conditions.", node_.getNamespace().c_str());
      initialize();
      publishStatus();
    }
  }

  template <class ActionSpec>
  ActionServer<ActionSpec>::ActionServer(ros::NodeHandle n, std::string name,
      boost::function<void (GoalHandle)> goal_cb,
      boost::function<void (GoalHandle)> cancel_cb,
      bool auto_start) :
    ActionServerBase<ActionSpec>(goal_cb, cancel_cb, auto_start),
    node_(n, name)
  {
    //if we're to autostart... then we'll initialize things
    if(this->started_){
      ROS_WARN_NAMED("actionlib", "You've passed in true for auto_start for the C++ action server at [%s]. You should always pass in false to avoid race conditions.", node_.getNamespace().c_str());
      initialize();
      publishStatus();
    }
  }

  template <class ActionSpec>
  ActionServer<ActionSpec>::ActionServer(ros::NodeHandle n, std::string name,
      boost::function<void (GoalHandle)> goal_cb,
      boost::function<void (GoalHandle)> cancel_cb) :
    ActionServerBase<ActionSpec>(goal_cb, cancel_cb, true),
    node_(n, name)
  {
    //if we're to autostart... then we'll initialize things
    if(this->started_){
      ROS_WARN_NAMED("actionlib", "You've passed in true for auto_start for the C++ action server at [%s]. You should always pass in false to avoid race conditions.", node_.getNamespace().c_str());
      initialize();
      publishStatus();
    }
  }

  template <class ActionSpec>
  ActionServer<ActionSpec>::ActionServer(ros::NodeHandle n, std::string name,
      boost::function<void (GoalHandle)> goal_cb,
      bool auto_start) :
    ActionServerBase<ActionSpec>(goal_cb, boost::function<void (GoalHandle)>(), auto_start),
    node_(n, name)
  {
    //if we're to autostart... then we'll initialize things
    if(this->started_){
      ROS_WARN_NAMED("actionlib", "You've passed in true for auto_start for the C++ action server at [%s]. You should always pass in false to avoid race conditions.", node_.getNamespace().c_str());
      initialize();
      publishStatus();
    }
  }

  template <class ActionSpec>
  ActionServer<ActionSpec>::~ActionServer()
  {
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::initialize()
  {
    result_pub_ = node_.advertise<ActionResult>("result", 50);
    feedback_pub_ = node_.advertise<ActionFeedback>("feedback", 50);
    status_pub_ = node_.advertise<actionlib_msgs::GoalStatusArray>("status", 50, true);

    //read the frequency with which to publish status from the parameter server
    //if not specified locally explicitly, use search param to find actionlib_status_frequency
    double status_frequency, status_list_timeout;
    if(!node_.getParam("status_frequency", status_frequency))
    {
      std::string status_frequency_param_name;
      if(!node_.searchParam("actionlib_status_frequency", status_frequency_param_name))
        status_frequency = 5.0;
      else
        node_.param(status_frequency_param_name, status_frequency, 5.0);
    }
    else
      ROS_WARN_NAMED("actionlib", "You're using the deprecated status_frequency parameter, please switch to actionlib_status_frequency.");

    node_.param("status_list_timeout", status_list_timeout, 5.0);

    this->status_list_timeout_ = ros::Duration(status_list_timeout);

    if(status_frequency > 0){
      status_timer_ = node_.createTimer(ros::Duration(1.0 / status_frequency),
          boost::bind(&ActionServer::publishStatus, this, _1));
    }

    goal_sub_ = node_.subscribe<ActionGoal>("goal", 50,
        boost::bind(&ActionServerBase<ActionSpec>::goalCallback, this, _1));

    cancel_sub_ = node_.subscribe<actionlib_msgs::GoalID>("cancel", 50,
        boost::bind(&ActionServerBase<ActionSpec>::cancelCallback, this, _1));

  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::publishResult(const actionlib_msgs::GoalStatus& status, const Result& result)
  {
    boost::recursive_mutex::scoped_lock lock(this->lock_);
    //we'll create a shared_ptr to pass to ROS to limit copying
    boost::shared_ptr<ActionResult> ar(new ActionResult);
    ar->header.stamp = ros::Time::now();
    ar->status = status;
    ar->result = result;
    ROS_DEBUG_NAMED("actionlib", "Publishing result for goal with id: %s and stamp: %.2f", status.goal_id.id.c_str(), status.goal_id.stamp.toSec());
    result_pub_.publish(ar);
    publishStatus();
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::publishFeedback(const actionlib_msgs::GoalStatus& status, const Feedback& feedback)
  {
    boost::recursive_mutex::scoped_lock lock(this->lock_);
    //we'll create a shared_ptr to pass to ROS to limit copying
    boost::shared_ptr<ActionFeedback> af(new ActionFeedback);
    af->header.stamp = ros::Time::now();
    af->status = status;
    af->feedback = feedback;
    ROS_DEBUG_NAMED("actionlib", "Publishing feedback for goal with id: %s and stamp: %.2f", status.goal_id.id.c_str(), status.goal_id.stamp.toSec());
    feedback_pub_.publish(af);
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::publishStatus(const ros::TimerEvent& e)
  {
    boost::recursive_mutex::scoped_lock lock(this->lock_);
    //we won't publish status unless we've been started
    if(!this->started_)
      return;

    publishStatus();
  }

  template <class ActionSpec>
  void ActionServer<ActionSpec>::publishStatus()
  {
    boost::recursive_mutex::scoped_lock lock(this->lock_);
    //build a status array
    actionlib_msgs::GoalStatusArray status_array;

    status_array.header.stamp = ros::Time::now();

    status_array.status_list.resize(this->status_list_.size());

    unsigned int i = 0;
    for(typename std::list<StatusTracker<ActionSpec> >::iterator it = this->status_list_.begin(); it != this->status_list_.end();){
      status_array.status_list[i] = (*it).status_;

      //check if the item is due for deletion from the status list
      if((*it).handle_destruction_time_ != ros::Time()
          && (*it).handle_destruction_time_ + this->status_list_timeout_ < ros::Time::now()){
        it = this->status_list_.erase(it);
      }
      else
        ++it;

      ++i;
    }

    status_pub_.publish(status_array);
  }
};
#endif
