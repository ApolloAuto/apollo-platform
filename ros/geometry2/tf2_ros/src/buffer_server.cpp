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
#include <tf2_ros/buffer_server.h>

namespace tf2_ros
{
  BufferServer::BufferServer(const Buffer& buffer, const std::string& ns, bool auto_start, ros::Duration check_period): 
    buffer_(buffer),
    server_(ros::NodeHandle(),
            ns,
            boost::bind(&BufferServer::goalCB, this, _1),
            boost::bind(&BufferServer::cancelCB, this, _1),
            auto_start)
  {
    ros::NodeHandle n;
    check_timer_ = n.createTimer(check_period, boost::bind(&BufferServer::checkTransforms, this, _1));
  }

  void BufferServer::checkTransforms(const ros::TimerEvent& e)
  {
    boost::mutex::scoped_lock l(mutex_);
    for(std::list<GoalInfo>::iterator it = active_goals_.begin(); it != active_goals_.end();)
    {
      GoalInfo& info = *it;

      //we want to lookup a transform if the time on the goal
      //has expired, or a transform is available
      if(canTransform(info.handle) || info.end_time < ros::Time::now())
      {
        tf2_msgs::LookupTransformResult result;

        //try to populate the result, catching exceptions if they occur
        try
        {
          result.transform = lookupTransform(info.handle);
        }
        catch (tf2::ConnectivityException &ex)
        {
          result.error.error = result.error.CONNECTIVITY_ERROR;
          result.error.error_string = ex.what();
        }
        catch (tf2::LookupException &ex)
        {
          result.error.error = result.error.LOOKUP_ERROR;
          result.error.error_string = ex.what();
        }
        catch (tf2::ExtrapolationException &ex)
        {
          result.error.error = result.error.EXTRAPOLATION_ERROR;
          result.error.error_string = ex.what();
        }
        catch (tf2::InvalidArgumentException &ex)
        {
          result.error.error = result.error.INVALID_ARGUMENT_ERROR;
          result.error.error_string = ex.what();
        }
        catch (tf2::TimeoutException &ex)
        {
          result.error.error = result.error.TIMEOUT_ERROR;
          result.error.error_string = ex.what();
        }
        catch (tf2::TransformException &ex)
        {
          result.error.error = result.error.TRANSFORM_ERROR;
          result.error.error_string = ex.what();
        }

        //make sure to pass the result to the client
        //even failed transforms are considered a success
        //since the request was successfully processed
        it = active_goals_.erase(it);
        info.handle.setSucceeded(result);
      }
      else
        ++it;
    }
  }

  void BufferServer::cancelCB(GoalHandle gh)
  {
    boost::mutex::scoped_lock l(mutex_);
    //we need to find the goal in the list and remove it... also setting it as canceled
    //if its not in the list, we won't do anything since it will have already been set
    //as completed
    for(std::list<GoalInfo>::iterator it = active_goals_.begin(); it != active_goals_.end();)
    {
      GoalInfo& info = *it;
      if(info.handle == gh)
      {
        it = active_goals_.erase(it);
        info.handle.setCanceled();
        return;
      }
      else
        ++it;
    }
  }

  void BufferServer::goalCB(GoalHandle gh)
  {
    //we'll accept all goals we get
    gh.setAccepted();

    //if the transform isn't immediately available, we'll push it onto our list to check
    //along with the time that the goal will end
    GoalInfo goal_info;
    goal_info.handle = gh;
    goal_info.end_time = ros::Time::now() + gh.getGoal()->timeout;

    //we can do a quick check here to see if the transform is valid
    //we'll also do this if the end time has been reached 
    if(canTransform(gh) || goal_info.end_time <= ros::Time::now())
    {
      tf2_msgs::LookupTransformResult result;
      try
      {
        result.transform = lookupTransform(gh);
      }
      catch (tf2::ConnectivityException &ex)
      {
        result.error.error = result.error.CONNECTIVITY_ERROR;
        result.error.error_string = ex.what();
      }
      catch (tf2::LookupException &ex)
      {
        result.error.error = result.error.LOOKUP_ERROR;
        result.error.error_string = ex.what();
      }
      catch (tf2::ExtrapolationException &ex)
      {
        result.error.error = result.error.EXTRAPOLATION_ERROR;
        result.error.error_string = ex.what();
      }
      catch (tf2::InvalidArgumentException &ex)
      {
        result.error.error = result.error.INVALID_ARGUMENT_ERROR;
        result.error.error_string = ex.what();
      }
      catch (tf2::TimeoutException &ex)
      {
        result.error.error = result.error.TIMEOUT_ERROR;
        result.error.error_string = ex.what();
      }
      catch (tf2::TransformException &ex)
      {
        result.error.error = result.error.TRANSFORM_ERROR;
        result.error.error_string = ex.what();
      }

      gh.setSucceeded(result);
      return;
    }

    boost::mutex::scoped_lock l(mutex_);
    active_goals_.push_back(goal_info);
  }

  bool BufferServer::canTransform(GoalHandle gh)
  {
    const tf2_msgs::LookupTransformGoal::ConstPtr& goal = gh.getGoal();

    //check whether we need to used the advanced or simple api
    if(!goal->advanced)
      return buffer_.canTransform(goal->target_frame, goal->source_frame, goal->source_time);

    return buffer_.canTransform(goal->target_frame, goal->target_time, 
        goal->source_frame, goal->source_time, goal->fixed_frame);
  }

  geometry_msgs::TransformStamped BufferServer::lookupTransform(GoalHandle gh)
  {
    const tf2_msgs::LookupTransformGoal::ConstPtr& goal = gh.getGoal();

    //check whether we need to used the advanced or simple api
    if(!goal->advanced)
      return buffer_.lookupTransform(goal->target_frame, goal->source_frame, goal->source_time);

    return buffer_.lookupTransform(goal->target_frame, goal->target_time, 
        goal->source_frame, goal->source_time, goal->fixed_frame);
  }

  void BufferServer::start()
  {
    server_.start();
  }

};
