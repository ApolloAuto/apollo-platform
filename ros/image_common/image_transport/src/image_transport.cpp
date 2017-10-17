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
#include "image_transport/publisher_plugin.h"
#include "image_transport/subscriber_plugin.h"
#include <pluginlib/class_loader.h>
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/erase.hpp>

namespace image_transport {

struct ImageTransport::Impl
{
  ros::NodeHandle nh_;
  PubLoaderPtr pub_loader_;
  SubLoaderPtr sub_loader_;
  
  Impl(const ros::NodeHandle& nh)
    : nh_(nh),
      pub_loader_( boost::make_shared<PubLoader>("image_transport", "image_transport::PublisherPlugin") ),
      sub_loader_( boost::make_shared<SubLoader>("image_transport", "image_transport::SubscriberPlugin") )
  {
  }
};

ImageTransport::ImageTransport(const ros::NodeHandle& nh)
  : impl_(new Impl(nh))
{
}

ImageTransport::~ImageTransport()
{
}

Publisher ImageTransport::advertise(const std::string& base_topic, uint32_t queue_size, bool latch)
{
  return advertise(base_topic, queue_size, SubscriberStatusCallback(),
                   SubscriberStatusCallback(), ros::VoidPtr(), latch);
}

Publisher ImageTransport::advertise(const std::string& base_topic, uint32_t queue_size,
                                    const SubscriberStatusCallback& connect_cb,
                                    const SubscriberStatusCallback& disconnect_cb,
                                    const ros::VoidPtr& tracked_object, bool latch)
{
  return Publisher(impl_->nh_, base_topic, queue_size, connect_cb, disconnect_cb, tracked_object, latch, impl_->pub_loader_);
}

Subscriber ImageTransport::subscribe(const std::string& base_topic, uint32_t queue_size,
                                     const boost::function<void(const sensor_msgs::ImageConstPtr&)>& callback,
                                     const ros::VoidPtr& tracked_object, const TransportHints& transport_hints)
{
  return Subscriber(impl_->nh_, base_topic, queue_size, callback, tracked_object, transport_hints, impl_->sub_loader_);
}

CameraPublisher ImageTransport::advertiseCamera(const std::string& base_topic, uint32_t queue_size, bool latch)
{
  return advertiseCamera(base_topic, queue_size,
                         SubscriberStatusCallback(), SubscriberStatusCallback(),
                         ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
                         ros::VoidPtr(), latch);
}

CameraPublisher ImageTransport::advertiseCamera(const std::string& base_topic, uint32_t queue_size,
                                                const SubscriberStatusCallback& image_connect_cb,
                                                const SubscriberStatusCallback& image_disconnect_cb,
                                                const ros::SubscriberStatusCallback& info_connect_cb,
                                                const ros::SubscriberStatusCallback& info_disconnect_cb,
                                                const ros::VoidPtr& tracked_object, bool latch)
{
  return CameraPublisher(*this, impl_->nh_, base_topic, queue_size, image_connect_cb, image_disconnect_cb,
                         info_connect_cb, info_disconnect_cb, tracked_object, latch);
}

CameraSubscriber ImageTransport::subscribeCamera(const std::string& base_topic, uint32_t queue_size,
                                                 const CameraSubscriber::Callback& callback,
                                                 const ros::VoidPtr& tracked_object,
                                                 const TransportHints& transport_hints)
{
  return CameraSubscriber(*this, impl_->nh_, base_topic, queue_size, callback, tracked_object, transport_hints);
}

std::vector<std::string> ImageTransport::getDeclaredTransports() const
{
  std::vector<std::string> transports = impl_->sub_loader_->getDeclaredClasses();
  // Remove the "_sub" at the end of each class name.
  BOOST_FOREACH(std::string& transport, transports) {
    transport = boost::erase_last_copy(transport, "_sub");
  }
  return transports;
}

std::vector<std::string> ImageTransport::getLoadableTransports() const
{
  std::vector<std::string> loadableTransports;

  BOOST_FOREACH( const std::string& transportPlugin, impl_->sub_loader_->getDeclaredClasses() )
  {
    // If the plugin loads without throwing an exception, add its
    // transport name to the list of valid plugins, otherwise ignore
    // it.
    try
    {
      boost::shared_ptr<image_transport::SubscriberPlugin> sub = impl_->sub_loader_->createInstance(transportPlugin);
      loadableTransports.push_back(boost::erase_last_copy(transportPlugin, "_sub")); // Remove the "_sub" at the end of each class name.
    }
    catch (const pluginlib::LibraryLoadException& e) {}
    catch (const pluginlib::CreateClassException& e) {}
  }

  return loadableTransports;

}

} //namespace image_transport
