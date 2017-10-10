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

#ifndef POLLED_CAMERA_PUBLICATION_SERVER_H
#define POLLED_CAMERA_PUBLICATION_SERVER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "polled_camera/GetPolledImage.h"

namespace polled_camera {

/**
 * \brief Manage image requests from one or more clients.
 *
 * Instances of polled_camera::PublicationServer should be created using one of
 * the overloads of polled_camera::advertise(). You must specify a driver
 * callback that populates the requested data:
\code
void callback(polled_camera::GetPolledImage::Request& req,
              polled_camera::GetPolledImage::Response& rsp,
              sensor_msgs::Image& image, sensor_msgs::CameraInfo& info)
{
  // Capture an image and fill in the Image and CameraInfo messages here.
  
  // On success, set rsp.success = true. rsp.timestamp will be filled in
  // automatically.
  
  // On failure, set rsp.success = false and fill rsp.status_message with an
  // informative error message.
}
\endcode
 */
class PublicationServer
{
public:
  typedef boost::function<void (polled_camera::GetPolledImage::Request&,
                                polled_camera::GetPolledImage::Response&,
                                sensor_msgs::Image&,
                                sensor_msgs::CameraInfo&)> DriverCallback;
  
  PublicationServer() {}

  /**
   * \brief Unadvertise the request service and shut down all published topics.
   */
  void shutdown();

  std::string getService() const;

  operator void*() const;
  bool operator< (const PublicationServer& rhs) const { return impl_ <  rhs.impl_; }
  bool operator==(const PublicationServer& rhs) const { return impl_ == rhs.impl_; }
  bool operator!=(const PublicationServer& rhs) const { return impl_ != rhs.impl_; }

private:
  PublicationServer(const std::string& service, ros::NodeHandle& nh,
                    const DriverCallback& cb, const ros::VoidPtr& tracked_object);

  class Impl;

  boost::shared_ptr<Impl> impl_;

  friend
  PublicationServer advertise(ros::NodeHandle&, const std::string&, const DriverCallback&,
                              const ros::VoidPtr&);
};

/**
 * \brief Advertise a polled image service, version for arbitrary boost::function object.
 */
PublicationServer advertise(ros::NodeHandle& nh, const std::string& service,
                            const PublicationServer::DriverCallback& cb,
                            const ros::VoidPtr& tracked_object = ros::VoidPtr());

/**
 * \brief Advertise a polled image service, version for class member function with bare pointer.
 */
template<class T>
PublicationServer advertise(ros::NodeHandle& nh, const std::string& service,
                            void(T::*fp)(polled_camera::GetPolledImage::Request&,
                                         polled_camera::GetPolledImage::Response&,
                                         sensor_msgs::Image&, sensor_msgs::CameraInfo&),
                            T* obj)
{
  return advertise(nh, service, boost::bind(fp, obj, _1, _2, _3, _4));
}

/**
 * \brief Advertise a polled image service, version for class member function with bare pointer.
 */
template<class T>
PublicationServer advertise(ros::NodeHandle& nh, const std::string& service,
                            void(T::*fp)(polled_camera::GetPolledImage::Request&,
                                         polled_camera::GetPolledImage::Response&,
                                         sensor_msgs::Image&, sensor_msgs::CameraInfo&),
                            const boost::shared_ptr<T>& obj)
{
  return advertise(nh, service, boost::bind(fp, obj.get(), _1, _2, _3, _4), obj);
}

} //namespace polled_camera

#endif
