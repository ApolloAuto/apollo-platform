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

#include "image_transport/image_transport.h"
#include "image_transport/camera_common.h"

namespace image_transport {

struct CameraPublisher::Impl
{
  Impl()
    : unadvertised_(false)
  {
  }

  ~Impl()
  {
    shutdown();
  }

  bool isValid() const
  {
    return !unadvertised_;
  }
  
  void shutdown()
  {
    if (!unadvertised_) {
      unadvertised_ = true;
      image_pub_.shutdown();
      info_pub_.shutdown();
    }
  }

  Publisher image_pub_;
  ros::Publisher info_pub_;
  bool unadvertised_;
  //double constructed_;
};

CameraPublisher::CameraPublisher(ImageTransport& image_it, ros::NodeHandle& info_nh,
                                 const std::string& base_topic, uint32_t queue_size,
                                 const SubscriberStatusCallback& image_connect_cb,
                                 const SubscriberStatusCallback& image_disconnect_cb,
                                 const ros::SubscriberStatusCallback& info_connect_cb,
                                 const ros::SubscriberStatusCallback& info_disconnect_cb,
                                 const ros::VoidPtr& tracked_object, bool latch)
  : impl_(new Impl)
{
  // Explicitly resolve name here so we compute the correct CameraInfo topic when the
  // image topic is remapped (#4539).
  std::string image_topic = info_nh.resolveName(base_topic);
  std::string info_topic = getCameraInfoTopic(image_topic);

  impl_->image_pub_ = image_it.advertise(image_topic, queue_size, image_connect_cb,
                                         image_disconnect_cb, tracked_object, latch);
  impl_->info_pub_ = info_nh.advertise<sensor_msgs::CameraInfo>(info_topic, queue_size, info_connect_cb,
                                                                info_disconnect_cb, tracked_object, latch);
}

uint32_t CameraPublisher::getNumSubscribers() const
{
  if (impl_ && impl_->isValid())
    return std::max(impl_->image_pub_.getNumSubscribers(), impl_->info_pub_.getNumSubscribers());
  return 0;
}

std::string CameraPublisher::getTopic() const
{
  if (impl_) return impl_->image_pub_.getTopic();
  return std::string();
}

std::string CameraPublisher::getInfoTopic() const
{
  if (impl_) return impl_->info_pub_.getTopic();
  return std::string();
}

void CameraPublisher::publish(const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& info) const
{
  if (!impl_ || !impl_->isValid()) {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid image_transport::CameraPublisher");
    return;
  }
  
  impl_->image_pub_.publish(image);
  impl_->info_pub_.publish(info);
}

void CameraPublisher::publish(const sensor_msgs::ImageConstPtr& image,
                              const sensor_msgs::CameraInfoConstPtr& info) const
{
  if (!impl_ || !impl_->isValid()) {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid image_transport::CameraPublisher");
    return;
  }
  
  impl_->image_pub_.publish(image);
  impl_->info_pub_.publish(info);
}

void CameraPublisher::publish(sensor_msgs::Image& image, sensor_msgs::CameraInfo& info,
                              ros::Time stamp) const
{
  if (!impl_ || !impl_->isValid()) {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid image_transport::CameraPublisher");
    return;
  }
  
  image.header.stamp = stamp;
  info.header.stamp = stamp;
  publish(image, info);
}

void CameraPublisher::shutdown()
{
  if (impl_) {
    impl_->shutdown();
    impl_.reset();
  }
}

CameraPublisher::operator void*() const
{
  return (impl_ && impl_->isValid()) ? (void*)1 : (void*)0;
}

} //namespace image_transport
