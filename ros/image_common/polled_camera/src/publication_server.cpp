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

#include "polled_camera/publication_server.h"
#include <image_transport/image_transport.h>

namespace polled_camera {

/// \cond

class PublicationServer::Impl
{
public:
  ros::ServiceServer srv_server_;
  DriverCallback driver_cb_;
  ros::VoidPtr tracked_object_;
  image_transport::ImageTransport it_;
  std::map<std::string, image_transport::CameraPublisher> client_map_;
  bool unadvertised_;
  double constructed_;
  
  Impl(const ros::NodeHandle& nh)
    : it_(nh),
      unadvertised_(false),
      constructed_(ros::WallTime::now().toSec())
  {
  }
  
  ~Impl()
  {
    if (ros::WallTime::now().toSec() - constructed_ < 0.001)
      ROS_WARN("PublicationServer destroyed immediately after creation. Did you forget to store the handle?");
    unadvertise();
  }

  bool isValid() const
  {
    return !unadvertised_;
  }

  void unadvertise()
  {
    if (!unadvertised_) {
      unadvertised_ = true;
      srv_server_.shutdown();
      client_map_.clear();
    }
  }

  bool requestCallback(polled_camera::GetPolledImage::Request& req,
                       polled_camera::GetPolledImage::Response& rsp)
  {
    std::string image_topic = req.response_namespace + "/image_raw";
    image_transport::CameraPublisher& pub = client_map_[image_topic];
    if (!pub) {
      // Create a latching camera publisher.
      pub = it_.advertiseCamera(image_topic, 1, image_transport::SubscriberStatusCallback(),
                                boost::bind(&Impl::disconnectCallback, this, _1),
                                ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
                                ros::VoidPtr(), true /*latch*/);
      ROS_INFO("Advertising %s", pub.getTopic().c_str());
    }

    // Correct zero binning values to one for convenience
    req.binning_x = std::max(req.binning_x, (uint32_t)1);
    req.binning_y = std::max(req.binning_y, (uint32_t)1);

    /// @todo Use pointers in prep for nodelet drivers?
    sensor_msgs::Image image;
    sensor_msgs::CameraInfo info;
    driver_cb_(req, rsp, image, info);
    
    if (rsp.success) {
      assert(image.header.stamp == info.header.stamp);
      rsp.stamp = image.header.stamp;
      pub.publish(image, info);
    }
    else {
      ROS_ERROR("Failed to capture requested image, status message: '%s'",
                rsp.status_message.c_str());
    }
    
    return true; // Success/failure indicated by rsp.success
  }

  void disconnectCallback(const image_transport::SingleSubscriberPublisher& ssp)
  {
    // Shut down the publication when the subscription count drops to zero.
    if (ssp.getNumSubscribers() == 0) {
      ROS_INFO("Shutting down %s", ssp.getTopic().c_str());
      client_map_.erase(ssp.getTopic());
    }
  }
};

/// \endcond

PublicationServer::PublicationServer(const std::string& service, ros::NodeHandle& nh,
                                     const DriverCallback& cb, const ros::VoidPtr& tracked_object)
  : impl_(new Impl(nh))
{
  impl_->driver_cb_ = cb;
  impl_->tracked_object_ = tracked_object;
  impl_->srv_server_ = nh.advertiseService<>(service, &Impl::requestCallback, impl_);
}

void PublicationServer::shutdown()
{
  if (impl_) impl_->unadvertise();
}

std::string PublicationServer::getService() const
{
  if (impl_) return impl_->srv_server_.getService();
  return std::string();
}

PublicationServer::operator void*() const
{
  return (impl_ && impl_->isValid()) ? (void*)1 : (void*)0;
}

PublicationServer advertise(ros::NodeHandle& nh, const std::string& service,
                            const PublicationServer::DriverCallback& cb,
                            const ros::VoidPtr& tracked_object)
{
  return PublicationServer(service, nh, cb, tracked_object);
}

} //namespace polled_camera
