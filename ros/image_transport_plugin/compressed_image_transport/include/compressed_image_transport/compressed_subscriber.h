/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 20012, Willow Garage, Inc.
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

#include "image_transport/simple_subscriber_plugin.h"
#include <sensor_msgs/CompressedImage.h>
#include <dynamic_reconfigure/server.h>
#include <compressed_image_transport/CompressedSubscriberConfig.h>

namespace compressed_image_transport {

class CompressedSubscriber : public image_transport::SimpleSubscriberPlugin<sensor_msgs::CompressedImage>
{
public:
  virtual ~CompressedSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "compressed";
  }

protected:
  // Overridden to set up reconfigure server
  virtual void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
          const Callback& callback, const ros::VoidPtr& tracked_object,
          const image_transport::TransportHints& transport_hints);


  virtual void internalCallback(const sensor_msgs::CompressedImageConstPtr& message,
                                const Callback& user_cb);

  typedef compressed_image_transport::CompressedSubscriberConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;
  int imdecode_flag_;

  void configCb(Config& config, uint32_t level);
};

} //namespace image_transport
