/*********************************************************************
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

#ifndef IMAGE_TRANSPORT_SUBSCRIBER_PLUGIN_H
#define IMAGE_TRANSPORT_SUBSCRIBER_PLUGIN_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/noncopyable.hpp>
#include "image_transport/transport_hints.h"

namespace image_transport {

/**
 * \brief Base class for plugins to Subscriber.
 */
class SubscriberPlugin : boost::noncopyable
{
public:
  typedef boost::function<void(const sensor_msgs::ImageConstPtr&)> Callback;
  
  virtual ~SubscriberPlugin() {}

  /**
   * \brief Get a string identifier for the transport provided by
   * this plugin.
   */
  virtual std::string getTransportName() const = 0;

  /**
   * \brief Subscribe to an image topic, version for arbitrary boost::function object.
   */
  void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                 const Callback& callback, const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                 const TransportHints& transport_hints = TransportHints())
  {
    return subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);
  }

  /**
   * \brief Subscribe to an image topic, version for bare function.
   */
  void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                 void(*fp)(const sensor_msgs::ImageConstPtr&),
                 const TransportHints& transport_hints = TransportHints())
  {
    return subscribe(nh, base_topic, queue_size,
                     boost::function<void(const sensor_msgs::ImageConstPtr&)>(fp),
                     ros::VoidPtr(), transport_hints);
  }

  /**
   * \brief Subscribe to an image topic, version for class member function with bare pointer.
   */
  template<class T>
  void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                 void(T::*fp)(const sensor_msgs::ImageConstPtr&), T* obj,
                 const TransportHints& transport_hints = TransportHints())
  {
    return subscribe(nh, base_topic, queue_size, boost::bind(fp, obj, _1), ros::VoidPtr(), transport_hints);
  }

  /**
   * \brief Subscribe to an image topic, version for class member function with shared_ptr.
   */
  template<class T>
  void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                 void(T::*fp)(const sensor_msgs::ImageConstPtr&),
                 const boost::shared_ptr<T>& obj,
                 const TransportHints& transport_hints = TransportHints())
  {
    return subscribe(nh, base_topic, queue_size, boost::bind(fp, obj.get(), _1), obj, transport_hints);
  }

  /**
   * \brief Get the transport-specific communication topic.
   */
  virtual std::string getTopic() const = 0;

  /**
   * \brief Returns the number of publishers this subscriber is connected to.
   */
  virtual uint32_t getNumPublishers() const = 0;

  /**
   * \brief Unsubscribe the callback associated with this SubscriberPlugin.
   */
  virtual void shutdown() = 0;

  /**
   * \brief Return the lookup name of the SubscriberPlugin associated with a specific
   * transport identifier.
   */
  static std::string getLookupName(const std::string& transport_type)
  {
    return "image_transport/" + transport_type + "_sub";
  }

protected:
  /**
   * \brief Subscribe to an image transport topic. Must be implemented by the subclass.
   */
  virtual void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                             const Callback& callback, const ros::VoidPtr& tracked_object,
                             const TransportHints& transport_hints) = 0;
};

} //namespace image_transport

#endif
