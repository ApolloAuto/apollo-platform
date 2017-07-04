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


#include <actionlib/client/connection_monitor.h>

#include <sstream>

using namespace std;
using namespace actionlib;


#define CONNECTION_DEBUG(fmt, ...) \
    ROS_DEBUG_NAMED("ConnectionMonitor", fmt,##__VA_ARGS__)

#define CONNECTION_WARN(fmt, ...) \
    ROS_WARN_NAMED("ConnectionMonitor", fmt,##__VA_ARGS__)


ConnectionMonitor::ConnectionMonitor(ros::Subscriber&feedback_sub, ros::Subscriber& result_sub)
  : feedback_sub_(feedback_sub), result_sub_(result_sub)
{
  status_received_ = false;
}

// ********* Goal Connections *********

void ConnectionMonitor::goalConnectCallback(const ros::SingleSubscriberPublisher& pub)
{
  boost::recursive_mutex::scoped_lock lock(data_mutex_);

  // Check if it's not in the list
  if (goalSubscribers_.find(pub.getSubscriberName()) == goalSubscribers_.end())
  {
    CONNECTION_DEBUG("goalConnectCallback: Adding [%s] to goalSubscribers", pub.getSubscriberName().c_str());
    goalSubscribers_[pub.getSubscriberName()] = 1;
  }
  else
  {
    CONNECTION_WARN("goalConnectCallback: Trying to add [%s] to goalSubscribers, but it is already in the goalSubscribers list", pub.getSubscriberName().c_str());
    goalSubscribers_[pub.getSubscriberName()]++;
  }
  CONNECTION_DEBUG("%s", goalSubscribersString().c_str());

  check_connection_condition_.notify_all();
}

void ConnectionMonitor::goalDisconnectCallback(const ros::SingleSubscriberPublisher& pub)
{
  boost::recursive_mutex::scoped_lock lock(data_mutex_);

  map<string, size_t>::iterator it;
  it = goalSubscribers_.find(pub.getSubscriberName());

  if (it == goalSubscribers_.end())
    CONNECTION_WARN("goalDisconnectCallback: Trying to remove [%s] to goalSubscribers, but it is not in the goalSubscribers list", pub.getSubscriberName().c_str());
  else
  {
    CONNECTION_DEBUG("goalDisconnectCallback: Removing [%s] from goalSubscribers", pub.getSubscriberName().c_str());
    goalSubscribers_[pub.getSubscriberName()]--;
    if (goalSubscribers_[pub.getSubscriberName()] == 0)
    {
      goalSubscribers_.erase(it);
    }
  }
  CONNECTION_DEBUG("%s", goalSubscribersString().c_str());
}

string ConnectionMonitor::goalSubscribersString()
{
  boost::recursive_mutex::scoped_lock lock(data_mutex_);
  ostringstream ss;
  ss << "Goal Subscribers (" << goalSubscribers_.size() << " total)";
  for (map<string, size_t>::iterator it=goalSubscribers_.begin(); it != goalSubscribers_.end(); it++)
    ss << "\n   - " << it->first;
  return ss.str();
}

// ********* Cancel Connections *********

void ConnectionMonitor::cancelConnectCallback(const ros::SingleSubscriberPublisher& pub)
{
  boost::recursive_mutex::scoped_lock lock(data_mutex_);

  // Check if it's not in the list
  if (cancelSubscribers_.find(pub.getSubscriberName()) == cancelSubscribers_.end())
  {
    CONNECTION_DEBUG("cancelConnectCallback: Adding [%s] to cancelSubscribers", pub.getSubscriberName().c_str());
    cancelSubscribers_[pub.getSubscriberName()] = 1;
  }
  else
  {
    CONNECTION_WARN("cancelConnectCallback: Trying to add [%s] to cancelSubscribers, but it is already in the cancelSubscribers list", pub.getSubscriberName().c_str());
    cancelSubscribers_[pub.getSubscriberName()]++;
  }
  CONNECTION_DEBUG("%s", cancelSubscribersString().c_str());

  check_connection_condition_.notify_all();
}

void ConnectionMonitor::cancelDisconnectCallback(const ros::SingleSubscriberPublisher& pub)
{
  boost::recursive_mutex::scoped_lock lock(data_mutex_);

  map<string, size_t>::iterator it;
  it = cancelSubscribers_.find(pub.getSubscriberName());

  if (it == cancelSubscribers_.end())
    CONNECTION_WARN("cancelDisconnectCallback: Trying to remove [%s] to cancelSubscribers, but it is not in the cancelSubscribers list", pub.getSubscriberName().c_str());
  else
  {
    CONNECTION_DEBUG("cancelDisconnectCallback: Removing [%s] from cancelSubscribers", pub.getSubscriberName().c_str());
    cancelSubscribers_[pub.getSubscriberName()]--;
    if (cancelSubscribers_[pub.getSubscriberName()] == 0)
    {
      cancelSubscribers_.erase(it);
    }
  }
  CONNECTION_DEBUG("%s", cancelSubscribersString().c_str());
}

string ConnectionMonitor::cancelSubscribersString()
{
  boost::recursive_mutex::scoped_lock lock(data_mutex_);

  ostringstream ss;
  ss << "cancel Subscribers (" << cancelSubscribers_.size() << " total)";
  for (map<string, size_t>::iterator it=cancelSubscribers_.begin(); it != cancelSubscribers_.end(); it++)
    ss << "\n   - " << it->first;
  return ss.str();
}

// ********* GoalStatus Connections *********
void ConnectionMonitor::processStatus(const actionlib_msgs::GoalStatusArrayConstPtr& status, const std::string& cur_status_caller_id)
{
  boost::recursive_mutex::scoped_lock lock(data_mutex_);

  if (status_received_)
  {
    if (status_caller_id_ != cur_status_caller_id)
    {
      CONNECTION_WARN("processStatus: Previously received status from [%s], but we now received status from [%s]. Did the ActionServer change?",
                   status_caller_id_.c_str(), cur_status_caller_id.c_str());
      status_caller_id_ = cur_status_caller_id;
    }
    latest_status_time_ = status->header.stamp;
  }
  else
  {
    CONNECTION_DEBUG("processStatus: Just got our first status message from the ActionServer at node [%s]", cur_status_caller_id.c_str());
    status_received_ = true;
    status_caller_id_ = cur_status_caller_id;
    latest_status_time_ = status->header.stamp;
  }

  check_connection_condition_.notify_all();
}

// ********* Connection logic *********
bool ConnectionMonitor::isServerConnected()
{
  boost::recursive_mutex::scoped_lock lock(data_mutex_);

  if (!status_received_)
  {
    CONNECTION_DEBUG("isServerConnected: Didn't receive status yet, so not connected yet");
    return false;
  }

  if(goalSubscribers_.find(status_caller_id_) == goalSubscribers_.end())
  {
    CONNECTION_DEBUG("isServerConnected: Server [%s] has not yet subscribed to the goal topic, so not connected yet", status_caller_id_.c_str());
    CONNECTION_DEBUG("%s", goalSubscribersString().c_str());
    return false;
  }

  if(cancelSubscribers_.find(status_caller_id_) == cancelSubscribers_.end())
  {
    CONNECTION_DEBUG("isServerConnected: Server [%s] has not yet subscribed to the cancel topic, so not connected yet", status_caller_id_.c_str());
    CONNECTION_DEBUG("%s", cancelSubscribersString().c_str());
    return false;
  }

  if(feedback_sub_.getNumPublishers() == 0)
  {
    CONNECTION_DEBUG("isServerConnected: Client has not yet connected to feedback topic of server [%s]", status_caller_id_.c_str());
    return false;
  }

  if(result_sub_.getNumPublishers() == 0)
  {
    CONNECTION_DEBUG("isServerConnected: Client has not yet connected to result topic of server [%s]", status_caller_id_.c_str());
    return false;
  }

  CONNECTION_DEBUG("isServerConnected: Server [%s] is fully connected", status_caller_id_.c_str());
  return true;
}

bool ConnectionMonitor::waitForActionServerToStart(const ros::Duration& timeout, const ros::NodeHandle& nh)
{
  if (timeout < ros::Duration(0,0))
    ROS_ERROR_NAMED("actionlib", "Timeouts can't be negative. Timeout is [%.2fs]", timeout.toSec());

  ros::Time timeout_time = ros::Time::now() + timeout;

  boost::recursive_mutex::scoped_lock lock(data_mutex_);

  if (isServerConnected())
    return true;

  // Hardcode how often we check for node.ok()
  ros::Duration loop_period = ros::Duration().fromSec(.5);

  while (nh.ok() && !isServerConnected())
  {
    // Determine how long we should wait
    ros::Duration time_left = timeout_time - ros::Time::now();

    // Check if we're past the timeout time
    if (timeout != ros::Duration(0,0) && time_left <= ros::Duration(0,0) )
      break;

    // Truncate the time left
    if (time_left > loop_period || timeout == ros::Duration())
      time_left = loop_period;

    check_connection_condition_.timed_wait(lock, boost::posix_time::milliseconds(time_left.toSec() * 1000.0f));
  }

  return isServerConnected();
}








