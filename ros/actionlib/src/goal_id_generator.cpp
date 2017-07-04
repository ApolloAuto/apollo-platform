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

#include <ros/ros.h>
#include <actionlib/goal_id_generator.h>
#include <boost/thread/mutex.hpp>

using namespace actionlib;

static boost::mutex s_goalcount_mutex_;
static unsigned int s_goalcount_ = 0;

GoalIDGenerator::GoalIDGenerator()
{
  setName(ros::this_node::getName());
}

GoalIDGenerator::GoalIDGenerator(const std::string& name)
{
  setName(name);
}

void GoalIDGenerator::setName(const std::string& name)
{
  name_ = name;
}

actionlib_msgs::GoalID GoalIDGenerator::generateID()
{
  actionlib_msgs::GoalID id;
  ros::Time cur_time = ros::Time::now();
  std::stringstream ss;

  ss << name_ << "-";

  {
    boost::mutex::scoped_lock lock(s_goalcount_mutex_);
    s_goalcount_++;
    ss << s_goalcount_ << "-";
  }

  ss << cur_time.sec << "." << cur_time.nsec;
  id.id = ss.str();
  id.stamp = cur_time;
  return id;
}
