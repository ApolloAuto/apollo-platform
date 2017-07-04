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

#ifndef ACTIONLIB_ACTION_CONNECTION_MONITOR_H_
#define ACTIONLIB_ACTION_CONNECTION_MONITOR_H_

#include <boost/thread/condition.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <set>
#include <map>

#include <actionlib/decl.h>

namespace actionlib
{

class ACTIONLIB_DECL ConnectionMonitor
{
public:
  ConnectionMonitor(ros::Subscriber&feedback_sub, ros::Subscriber& result_sub);

  void goalConnectCallback(const ros::SingleSubscriberPublisher& pub);

  void goalDisconnectCallback(const ros::SingleSubscriberPublisher& pub);

  void cancelConnectCallback(const ros::SingleSubscriberPublisher& pub);

  void cancelDisconnectCallback(const ros::SingleSubscriberPublisher& pub);

  void processStatus(const actionlib_msgs::GoalStatusArrayConstPtr& status, const std::string& caller_id);

  bool waitForActionServerToStart(const ros::Duration& timeout = ros::Duration(0,0), const ros::NodeHandle& nh = ros::NodeHandle() );
  bool isServerConnected();

private:

  // status stuff
  std::string status_caller_id_;
  bool status_received_;
  ros::Time latest_status_time_;

  boost::condition check_connection_condition_;

  boost::recursive_mutex data_mutex_;
  std::map<std::string, size_t> goalSubscribers_;
  std::map<std::string, size_t> cancelSubscribers_;

  std::string goalSubscribersString();
  std::string cancelSubscribersString();

  ros::Subscriber& feedback_sub_;
  ros::Subscriber& result_sub_;
};

}

#endif
