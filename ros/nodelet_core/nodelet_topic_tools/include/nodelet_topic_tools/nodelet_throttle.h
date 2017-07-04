#ifndef _NODELET_TOPIC_TOOLS_NODELET_THROTTLE_H_
#define _NODELET_TOPIC_TOOLS_NODELET_THROTTLE_H_

/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <nodelet_topic_tools/NodeletThrottleConfig.h>

#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

namespace nodelet_topic_tools
{

template<typename M>
class NodeletThrottle : public nodelet::Nodelet
{
public:
  //Constructor
  NodeletThrottle(): max_update_rate_(0)
  {
  };

  ~NodeletThrottle()
  {
      delete srv_;
  }

private:
  ros::Time last_update_;
  double max_update_rate_;
  boost::mutex connect_mutex_;
  dynamic_reconfigure::Server<nodelet_topic_tools::NodeletThrottleConfig>* srv_;

  virtual void onInit()
  {
    nh_ = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    srv_ = new dynamic_reconfigure::Server<nodelet_topic_tools::NodeletThrottleConfig>(private_nh);
    dynamic_reconfigure::Server<nodelet_topic_tools::NodeletThrottleConfig>::CallbackType f = boost::bind(&NodeletThrottle::reconfigure, this, _1, _2);
    srv_->setCallback(f);

    // Lazy subscription to topic
    ros::AdvertiseOptions publisher_ao = ros::AdvertiseOptions::create<M>(
      "topic_out", 10,
      boost::bind( &NodeletThrottle::connectCB, this),
      boost::bind( &NodeletThrottle::disconnectCB, this), ros::VoidPtr(), nh_.getCallbackQueue());

    // Need to create the publisher with connection mutex
    // connectCB can be called before the publisher is created in nodelet
    // which means no topics will connect
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_ = nh_.advertise(publisher_ao);
  };

  void callback(const boost::shared_ptr<const M>& cloud)
  {
    if (max_update_rate_ > 0.0)
    {
      NODELET_DEBUG("update set to %f", max_update_rate_);
      if ( last_update_ + ros::Duration(1.0/max_update_rate_) > ros::Time::now())
      {
        NODELET_DEBUG("throttle last update at %f skipping", last_update_.toSec());
        return;
      }
    }

    last_update_ = ros::Time::now();
    pub_.publish(cloud);
  }

  void reconfigure(nodelet_topic_tools::NodeletThrottleConfig &config, uint32_t level)
  {
      max_update_rate_ = config.update_rate;
  }

  void connectCB() {
      boost::lock_guard<boost::mutex> lock(connect_mutex_);
      if (pub_.getNumSubscribers() > 0) {
          NODELET_DEBUG("Connecting to topic");
          sub_ = nh_.subscribe<M>("topic_in", 10, &NodeletThrottle::callback, this);
      }
  }

  void disconnectCB() {
      boost::lock_guard<boost::mutex> lock(connect_mutex_);
      if (pub_.getNumSubscribers() == 0) {
          NODELET_DEBUG("Unsubscribing from topic.");
          sub_.shutdown();
      }
  }

  ros::NodeHandle nh_;
  ros::Publisher  pub_;
  ros::Subscriber sub_;
};

} // namespace

#endif // guard
