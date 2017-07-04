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
#ifndef ACTIONLIB_STATUS_TRACKER_H_
#define ACTIONLIB_STATUS_TRACKER_H_

#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib/action_definition.h>

#include <actionlib/goal_id_generator.h>

namespace actionlib {

  /**
   * @class StatusTracker
   * @brief A class for storing the status of each goal the action server
   * is currently working on
   */
  template <class ActionSpec>
    class StatusTracker {
      private:
        //generates typedefs that we'll use to make our lives easier
        ACTION_DEFINITION(ActionSpec);

      public:
        StatusTracker(const actionlib_msgs::GoalID& goal_id, unsigned int status);

        StatusTracker(const boost::shared_ptr<const ActionGoal>& goal);

        boost::shared_ptr<const ActionGoal> goal_;
        boost::weak_ptr<void> handle_tracker_;
        actionlib_msgs::GoalStatus status_;
        ros::Time handle_destruction_time_;

      private:
        GoalIDGenerator id_generator_;
    };
};

//include the implementation
#include <actionlib/server/status_tracker_imp.h>
#endif
