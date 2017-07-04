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
#ifndef ACTIONLIB_STATUS_TRACKER_IMP_H_
#define ACTIONLIB_STATUS_TRACKER_IMP_H_
namespace actionlib {
  template <class ActionSpec>
  StatusTracker<ActionSpec>::StatusTracker(const actionlib_msgs::GoalID& goal_id, unsigned int status){
    //set the goal id and status appropriately
    status_.goal_id = goal_id;
    status_.status = status;
  }

  template <class ActionSpec>
  StatusTracker<ActionSpec>::StatusTracker(const boost::shared_ptr<const ActionGoal>& goal)
    : goal_(goal) {
      //set the goal_id from the message
      status_.goal_id = goal_->goal_id;

      //initialize the status of the goal to pending
      status_.status = actionlib_msgs::GoalStatus::PENDING;

      //if the goal id is zero, then we need to make up an id for the goal
      if(status_.goal_id.id == ""){
        status_.goal_id = id_generator_.generateID();
      }

      //if the timestamp of the goal is zero, then we'll set it to now()
      if(status_.goal_id.stamp == ros::Time()){
        status_.goal_id.stamp = ros::Time::now();
      }
    }
};
#endif
