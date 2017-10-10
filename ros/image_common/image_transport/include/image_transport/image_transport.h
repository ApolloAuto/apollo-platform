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

#ifndef IMAGE_TRANSPORT_IMAGE_TRANSPORT_H
#define IMAGE_TRANSPORT_IMAGE_TRANSPORT_H

#include "image_transport/publisher.h"
#include "image_transport/subscriber.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/camera_subscriber.h"

namespace image_transport {

/**
 * \brief Advertise and subscribe to image topics.
 *
 * ImageTransport is analogous to ros::NodeHandle in that it contains advertise() and
 * subscribe() functions for creating advertisements and subscriptions of image topics.
 */
class ImageTransport
{
public:
  explicit ImageTransport(const ros::NodeHandle& nh);

  ~ImageTransport();

  /*!
   * \brief Advertise an image topic, simple version.
   */
  Publisher advertise(const std::string& base_topic, uint32_t queue_size, bool latch = false);

  /*!
   * \brief Advertise an image topic with subcriber status callbacks.
   */
  Publisher advertise(const std::string& base_topic, uint32_t queue_size,
                      const SubscriberStatusCallback& connect_cb,
                      const SubscriberStatusCallback& disconnect_cb = SubscriberStatusCallback(),
                      const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = false);

  /**
   * \brief Subscribe to an image topic, version for arbitrary boost::function object.
   */
  Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                       const boost::function<void(const sensor_msgs::ImageConstPtr&)>& callback,
                       const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                       const TransportHints& transport_hints = TransportHints());

  /**
   * \brief Subscribe to an image topic, version for bare function.
   */
  Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                       void(*fp)(const sensor_msgs::ImageConstPtr&),
                       const TransportHints& transport_hints = TransportHints())
  {
    return subscribe(base_topic, queue_size,
                     boost::function<void(const sensor_msgs::ImageConstPtr&)>(fp),
                     ros::VoidPtr(), transport_hints);
  }

  /**
   * \brief Subscribe to an image topic, version for class member function with bare pointer.
   */
  template<class T>
  Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                       void(T::*fp)(const sensor_msgs::ImageConstPtr&), T* obj,
                       const TransportHints& transport_hints = TransportHints())
  {
    return subscribe(base_topic, queue_size, boost::bind(fp, obj, _1), ros::VoidPtr(), transport_hints);
  }

  /**
   * \brief Subscribe to an image topic, version for class member function with shared_ptr.
   */
  template<class T>
  Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                       void(T::*fp)(const sensor_msgs::ImageConstPtr&),
                       const boost::shared_ptr<T>& obj,
                       const TransportHints& transport_hints = TransportHints())
  {
    return subscribe(base_topic, queue_size, boost::bind(fp, obj.get(), _1), obj, transport_hints);
  }

  /*!
   * \brief Advertise a synchronized camera raw image + info topic pair, simple version.
   */
  CameraPublisher advertiseCamera(const std::string& base_topic, uint32_t queue_size, bool latch = false);

  /*!
   * \brief Advertise a synchronized camera raw image + info topic pair with subscriber status
   * callbacks.
   */
  CameraPublisher advertiseCamera(const std::string& base_topic, uint32_t queue_size,
                                  const SubscriberStatusCallback& image_connect_cb,
                                  const SubscriberStatusCallback& image_disconnect_cb = SubscriberStatusCallback(),
                                  const ros::SubscriberStatusCallback& info_connect_cb = ros::SubscriberStatusCallback(),
                                  const ros::SubscriberStatusCallback& info_disconnect_cb = ros::SubscriberStatusCallback(),
                                  const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = false);

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for arbitrary
   * boost::function object.
   *
   * This version assumes the standard topic naming scheme, where the info topic is
   * named "camera_info" in the same namespace as the base image topic.
   */
  CameraSubscriber subscribeCamera(const std::string& base_topic, uint32_t queue_size,
                                   const CameraSubscriber::Callback& callback,
                                   const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                                   const TransportHints& transport_hints = TransportHints());

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for bare function.
   */
  CameraSubscriber subscribeCamera(const std::string& base_topic, uint32_t queue_size,
                                   void(*fp)(const sensor_msgs::ImageConstPtr&,
                                             const sensor_msgs::CameraInfoConstPtr&),
                                   const TransportHints& transport_hints = TransportHints())
  {
    return subscribeCamera(base_topic, queue_size, CameraSubscriber::Callback(fp), ros::VoidPtr(),
                           transport_hints);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member
   * function with bare pointer.
   */
  template<class T>
  CameraSubscriber subscribeCamera(const std::string& base_topic, uint32_t queue_size,
                                   void(T::*fp)(const sensor_msgs::ImageConstPtr&,
                                                const sensor_msgs::CameraInfoConstPtr&), T* obj,
                                   const TransportHints& transport_hints = TransportHints())
  {
    return subscribeCamera(base_topic, queue_size, boost::bind(fp, obj, _1, _2), ros::VoidPtr(),
                           transport_hints);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member
   * function with shared_ptr.
   */
  template<class T>
  CameraSubscriber subscribeCamera(const std::string& base_topic, uint32_t queue_size,
                                   void(T::*fp)(const sensor_msgs::ImageConstPtr&,
                                                const sensor_msgs::CameraInfoConstPtr&),
                                   const boost::shared_ptr<T>& obj,
                                   const TransportHints& transport_hints = TransportHints())
  {
    return subscribeCamera(base_topic, queue_size, boost::bind(fp, obj.get(), _1, _2), obj,
                           transport_hints);
  }

  /**
   * \brief Returns the names of all transports declared in the system. Declared
   * transports are not necessarily built or loadable.
   */
  std::vector<std::string> getDeclaredTransports() const;

  /**
   * \brief Returns the names of all transports that are loadable in the system.
   */
  std::vector<std::string> getLoadableTransports() const;

private:
  struct Impl;
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl> ImplWPtr;

  ImplPtr impl_;
};

} //namespace image_transport

#endif
