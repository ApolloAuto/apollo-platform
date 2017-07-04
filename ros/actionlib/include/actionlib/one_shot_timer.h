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

#ifndef ACTION_TOOLS_ROBUST_ONE_SHOT_TIMER_H_
#define ACTION_TOOLS_ROBUST_ONE_SHOT_TIMER_H_

#include "ros/ros.h"
#include "boost/bind.hpp"

//! Horrible hack until ROS Supports this (ROS Trac #1387)
class OneShotTimer
{
public:
  OneShotTimer() : active_(false)  { }

  void cb(const ros::TimerEvent& e)
  {
    if (active_)
    {
      active_ = false;

      if (callback_)
        callback_(e);
      else
        ROS_ERROR_NAMED("actionlib", "Got a NULL Timer OneShotTimer Callback");
    }
  }

  boost::function<void (const ros::TimerEvent& e)> getCb()
  {
    return boost::bind(&OneShotTimer::cb, this, _1);
  }

  void registerOneShotCb(boost::function<void (const ros::TimerEvent& e)> callback)
  {
    callback_ = callback;
  }

  void stop()
  {
    //timer_.stop();
    active_ = false;
  }

  const ros::Timer& operator=(const ros::Timer& rhs)
  {
    active_ = true;
    timer_ = rhs;
    return timer_;
  }
private:
  ros::Timer timer_;
  bool active_;
  boost::function<void (const ros::TimerEvent& e)> callback_;
};


#endif
