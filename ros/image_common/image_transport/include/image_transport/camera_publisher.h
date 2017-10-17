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

#ifndef IMAGE_TRANSPORT_CAMERA_PUBLISHER_H
#define IMAGE_TRANSPORT_CAMERA_PUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "image_transport/single_subscriber_publisher.h"

namespace image_transport {

class ImageTransport;

/**
 * \brief Manages advertisements for publishing camera images.
 *
 * CameraPublisher is a convenience class for publishing synchronized image and
 * camera info topics using the standard topic naming convention, where the info
 * topic name is "camera_info" in the same namespace as the base image topic.
 *
 * On the client side, CameraSubscriber simplifies subscribing to camera images.
 *
 * A CameraPublisher should always be created through a call to
 * ImageTransport::advertiseCamera(), or copied from one that was.
 * Once all copies of a specific CameraPublisher go out of scope, any subscriber callbacks
 * associated with that handle will stop being called. Once all CameraPublisher for a
 * given base topic go out of scope the topic (and all subtopics) will be unadvertised.
 */
class CameraPublisher
{
public:
  CameraPublisher() {}

  /*!
   * \brief Returns the number of subscribers that are currently connected to
   * this CameraPublisher.
   *
   * Returns max(image topic subscribers, info topic subscribers).
   */
  uint32_t getNumSubscribers() const;

  /*!
   * \brief Returns the base (image) topic of this CameraPublisher.
   */
  std::string getTopic() const;

  /**
   * \brief Returns the camera info topic of this CameraPublisher.
   */
  std::string getInfoTopic() const;

  /*!
   * \brief Publish an (image, info) pair on the topics associated with this CameraPublisher.
   */
  void publish(const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& info) const;

  /*!
   * \brief Publish an (image, info) pair on the topics associated with this CameraPublisher.
   */
  void publish(const sensor_msgs::ImageConstPtr& image,
               const sensor_msgs::CameraInfoConstPtr& info) const;

  /*!
   * \brief Publish an (image, info) pair with given timestamp on the topics associated with
   * this CameraPublisher.
   *
   * Convenience version, which sets the timestamps of both image and info to stamp before
   * publishing.
   */
  void publish(sensor_msgs::Image& image, sensor_msgs::CameraInfo& info, ros::Time stamp) const;

  /*!
   * \brief Shutdown the advertisements associated with this Publisher.
   */
  void shutdown();

  operator void*() const;
  bool operator< (const CameraPublisher& rhs) const { return impl_ <  rhs.impl_; }
  bool operator!=(const CameraPublisher& rhs) const { return impl_ != rhs.impl_; }
  bool operator==(const CameraPublisher& rhs) const { return impl_ == rhs.impl_; }

private:
  CameraPublisher(ImageTransport& image_it, ros::NodeHandle& info_nh,
                  const std::string& base_topic, uint32_t queue_size,
                  const SubscriberStatusCallback& image_connect_cb,
                  const SubscriberStatusCallback& image_disconnect_cb,
                  const ros::SubscriberStatusCallback& info_connect_cb,
                  const ros::SubscriberStatusCallback& info_disconnect_cb,
                  const ros::VoidPtr& tracked_object, bool latch);
  
  struct Impl;
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl> ImplWPtr;
  
  ImplPtr impl_;

  friend class ImageTransport;
};

} //namespace image_transport

#endif
