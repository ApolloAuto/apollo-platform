/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
#include <tf2_ros/buffer_client.h>

namespace tf2_ros
{
  BufferClient::BufferClient(std::string ns, double check_frequency, ros::Duration timeout_padding): 
    client_(ns), 
    check_frequency_(check_frequency),
    timeout_padding_(timeout_padding)
  {
  }

  geometry_msgs::TransformStamped BufferClient::lookupTransform(const std::string& target_frame, const std::string& source_frame,
      const ros::Time& time, const ros::Duration timeout) const
  {
    //populate the goal message
    tf2_msgs::LookupTransformGoal goal;
    goal.target_frame = target_frame;
    goal.source_frame = source_frame;
    goal.source_time = time;
    goal.timeout = timeout;
    goal.advanced = false;

    return processGoal(goal);
  }

  geometry_msgs::TransformStamped BufferClient::lookupTransform(const std::string& target_frame, const ros::Time& target_time,
      const std::string& source_frame, const ros::Time& source_time,
      const std::string& fixed_frame, const ros::Duration timeout) const
  {
    //populate the goal message
    tf2_msgs::LookupTransformGoal goal;
    goal.target_frame = target_frame;
    goal.source_frame = source_frame;
    goal.source_time = source_time;
    goal.timeout = timeout;
    goal.target_time = target_time;
    goal.fixed_frame = fixed_frame;
    goal.advanced = true;

    return processGoal(goal);
  }

  geometry_msgs::TransformStamped BufferClient::processGoal(const tf2_msgs::LookupTransformGoal& goal) const
  {
    client_.sendGoal(goal);
    ros::Rate r(check_frequency_);
    bool timed_out = false;
    ros::Time start_time = ros::Time::now();
    while(ros::ok() && !client_.getState().isDone() && !timed_out)
    {
      timed_out = ros::Time::now() > start_time + goal.timeout + timeout_padding_;
      r.sleep();
    }

    //this shouldn't happen, but could in rare cases where the server hangs
    if(timed_out)
    {
      //make sure to cancel the goal the server is pursuing
      client_.cancelGoal();
      throw tf2::TimeoutException("The LookupTransform goal sent to the BufferServer did not come back in the specified time. Something is likely wrong with the server.");
    }

    if(client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      throw tf2::TimeoutException("The LookupTransform goal sent to the BufferServer did not come back with SUCCEEDED status. Something is likely wrong with the server.");

    //process the result for errors and return it
    return processResult(*client_.getResult());
  }

  geometry_msgs::TransformStamped BufferClient::processResult(const tf2_msgs::LookupTransformResult& result) const
  {
    //if there's no error, then we'll just return the transform
    if(result.error.error != result.error.NO_ERROR){
      //otherwise, we'll have to throw the appropriate exception
      if(result.error.error == result.error.LOOKUP_ERROR)
        throw tf2::LookupException(result.error.error_string);

      if(result.error.error == result.error.CONNECTIVITY_ERROR)
        throw tf2::ConnectivityException(result.error.error_string);

      if(result.error.error == result.error.EXTRAPOLATION_ERROR)
        throw tf2::ExtrapolationException(result.error.error_string);

      if(result.error.error == result.error.INVALID_ARGUMENT_ERROR)
        throw tf2::InvalidArgumentException(result.error.error_string);

      if(result.error.error == result.error.TIMEOUT_ERROR)
        throw tf2::TimeoutException(result.error.error_string);

      throw tf2::TransformException(result.error.error_string);
    }

    return result.transform;
  }

  bool BufferClient::canTransform(const std::string& target_frame, const std::string& source_frame, 
        const ros::Time& time, const ros::Duration timeout, std::string* errstr) const
  {
    try
    {
      lookupTransform(target_frame, source_frame, time, timeout);
      return true;
    }
    catch(tf2::TransformException& ex)
    {
      if(errstr)
        *errstr = ex.what();
      return false;
    }
  }

  bool BufferClient::canTransform(const std::string& target_frame, const ros::Time& target_time,
        const std::string& source_frame, const ros::Time& source_time,
        const std::string& fixed_frame, const ros::Duration timeout, std::string* errstr) const
  {
    try
    {
      lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
      return true;
    }
    catch(tf2::TransformException& ex)
    {
      if(errstr)
        *errstr = ex.what();
      return false;
    }
  }
};
