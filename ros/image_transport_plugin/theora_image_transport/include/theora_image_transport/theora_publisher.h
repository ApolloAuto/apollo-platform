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

#include <image_transport/simple_publisher_plugin.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <dynamic_reconfigure/server.h>
#include <theora_image_transport/TheoraPublisherConfig.h>
#include <theora_image_transport/Packet.h>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

namespace theora_image_transport {

class TheoraPublisher : public image_transport::SimplePublisherPlugin<theora_image_transport::Packet>
{
public:
  TheoraPublisher();

  ~TheoraPublisher();
  
  // Return the system unique string representing the theora transport type
  virtual std::string getTransportName() const { return "theora"; }

protected:
  // Overridden to tweak arguments and set up reconfigure server
  virtual void advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                             const image_transport::SubscriberStatusCallback  &user_connect_cb,
                             const image_transport::SubscriberStatusCallback  &user_disconnect_cb,
                             const ros::VoidPtr &tracked_object, bool latch);
  
  // Callback to send header packets to new clients
  virtual void connectCallback(const ros::SingleSubscriberPublisher& pub);

  // Main publish function
  virtual void publish(const sensor_msgs::Image& message,
                       const PublishFn& publish_fn) const;

  // Dynamic reconfigure support
  typedef theora_image_transport::TheoraPublisherConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  void configCb(Config& config, uint32_t level);

  // Utility functions
  bool ensureEncodingContext(const sensor_msgs::Image& image, const PublishFn& publish_fn) const;
  void oggPacketToMsg(const std_msgs::Header& header, const ogg_packet &oggpacket,
                      theora_image_transport::Packet &msg) const;
  void updateKeyframeFrequency() const;

  // Some data is preserved across calls to publish(), but from the user's perspective publish() is
  // "logically const"
  mutable cv_bridge::CvImage img_image_;
  mutable th_info encoder_setup_;
  mutable ogg_uint32_t keyframe_frequency_;
  mutable boost::shared_ptr<th_enc_ctx> encoding_context_;
  mutable std::vector<theora_image_transport::Packet> stream_header_;
};

} //namespace compressed_image_transport
