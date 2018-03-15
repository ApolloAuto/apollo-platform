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

#include "theora_image_transport/theora_publisher.h"
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <vector>
#include <cstdio> //for memcpy

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

namespace theora_image_transport {

TheoraPublisher::TheoraPublisher()
{
  // Initialize info structure fields that don't change
  th_info_init(&encoder_setup_);
  
  encoder_setup_.pic_x = 0;
  encoder_setup_.pic_y = 0;
  encoder_setup_.colorspace = TH_CS_UNSPECIFIED;
  encoder_setup_.pixel_fmt = TH_PF_420; // See bottom of http://www.theora.org/doc/libtheora-1.1beta1/codec_8h.html
  encoder_setup_.aspect_numerator = 1;
  encoder_setup_.aspect_denominator = 1;
  encoder_setup_.fps_numerator = 1; // don't know the frame rate ahead of time
  encoder_setup_.fps_denominator = 1;
  encoder_setup_.keyframe_granule_shift = 6; // A good default for streaming applications
  // Note: target_bitrate and quality set to correct values in configCb
  encoder_setup_.target_bitrate = -1;
  encoder_setup_.quality = -1;
}

TheoraPublisher::~TheoraPublisher()
{
  th_info_clear(&encoder_setup_);
}

void TheoraPublisher::advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                                    const image_transport::SubscriberStatusCallback  &user_connect_cb,
                                    const image_transport::SubscriberStatusCallback  &user_disconnect_cb,
                                    const ros::VoidPtr &tracked_object, bool latch)
{
  // queue_size doesn't account for the 3 header packets, so we correct (with a little extra) here.
  queue_size += 4;
  // Latching doesn't make a lot of sense with this transport. Could try to save the last keyframe,
  // but do you then send all following delta frames too?
  latch = false;
  typedef image_transport::SimplePublisherPlugin<theora_image_transport::Packet> Base;
  Base::advertiseImpl(nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb, tracked_object, latch);

  // Set up reconfigure server for this topic
  reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
  ReconfigureServer::CallbackType f = boost::bind(&TheoraPublisher::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);
}

void TheoraPublisher::configCb(Config& config, uint32_t level)
{
  // target_bitrate must be 0 if we're using quality.
  long bitrate = 0;
  if (config.optimize_for == theora_image_transport::TheoraPublisher_Bitrate)
    bitrate = config.target_bitrate;
  bool update_bitrate = bitrate && encoder_setup_.target_bitrate != bitrate;
  bool update_quality = !bitrate && ((encoder_setup_.quality != config.quality) || encoder_setup_.target_bitrate > 0);
  encoder_setup_.quality = config.quality;
  encoder_setup_.target_bitrate = bitrate;
  keyframe_frequency_ = config.keyframe_frequency;
  
  if (encoding_context_) {
    int err = 0;
    // libtheora 1.1 lets us change quality or bitrate on the fly, 1.0 does not.
#ifdef TH_ENCCTL_SET_BITRATE
    if (update_bitrate) {
      err = th_encode_ctl(encoding_context_.get(), TH_ENCCTL_SET_BITRATE, &bitrate, sizeof(long));
      if (err)
        ROS_ERROR("Failed to update bitrate dynamically");
    }
#else
    err |= update_bitrate;
#endif

#ifdef TH_ENCCTL_SET_QUALITY
    if (update_quality) {
      err = th_encode_ctl(encoding_context_.get(), TH_ENCCTL_SET_QUALITY, &config.quality, sizeof(int));
      // In 1.1 above call will fail if a bitrate has previously been set. That restriction may
      // be relaxed in a future version. Complain on other failures.
      if (err && err != TH_EINVAL)
        ROS_ERROR("Failed to update quality dynamically");
    }
#else
    err |= update_quality;
#endif

    // If unable to change parameters dynamically, just create a new encoding context.
    if (err) {
      encoding_context_.reset();
    }
    // Otherwise, do the easy updates and keep going!
    else {
      updateKeyframeFrequency();
      config.keyframe_frequency = keyframe_frequency_; // In case desired value was unattainable
    }
  }
}

void TheoraPublisher::connectCallback(const ros::SingleSubscriberPublisher& pub)
{
  // Send the header packets to new subscribers
  for (unsigned int i = 0; i < stream_header_.size(); i++) {
    pub.publish(stream_header_[i]);
  }
}

static void cvToTheoraPlane(cv::Mat& mat, th_img_plane& plane)
{
  plane.width  = mat.cols;
  plane.height = mat.rows;
  plane.stride = mat.step;
  plane.data   = mat.data;
}

void TheoraPublisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
{
  if (!ensureEncodingContext(message, publish_fn))
    return;
  //return;
  /// @todo fromImage is deprecated
  /// @todo Optimized gray-scale path, rgb8
  /// @todo fromImage can throw cv::Exception on bayer encoded images

  cv_bridge::CvImageConstPtr cv_image_ptr;
  try
  {
    // conversion necessary
    cv_image_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: '%s'", e.what());
    return;
  }
  catch (cv::Exception& e)
  {
    ROS_ERROR("OpenCV exception: '%s'", e.what());
    return;
  }

  if (cv_image_ptr == 0) {
    ROS_ERROR("Unable to convert from '%s' to 'bgr8'", message.encoding.c_str());
    return;
  }

  const cv::Mat bgr = cv_image_ptr->image;

  cv::Mat bgr_padded;
  int frame_width = encoder_setup_.frame_width, frame_height = encoder_setup_.frame_height;
  if (frame_width == bgr.cols && frame_height == bgr.rows) {
    bgr_padded = bgr;
  }
  else {
    bgr_padded = cv::Mat::zeros(frame_height, frame_width, bgr.type());
    cv::Mat pic_roi = bgr_padded(cv::Rect(0, 0, bgr.cols, bgr.rows));
    bgr.copyTo(pic_roi);
  }

  // Convert image to Y'CbCr color space used by Theora
  cv::Mat ycrcb;
  cv::cvtColor(bgr_padded, ycrcb, cv::COLOR_BGR2YCrCb);
  
  // Split channels
  cv::Mat ycrcb_planes[3];
  cv::split(ycrcb, ycrcb_planes);

  // Use Y as-is but subsample chroma channels
  cv::Mat y = ycrcb_planes[0], cr, cb;
  cv::pyrDown(ycrcb_planes[1], cr);
  cv::pyrDown(ycrcb_planes[2], cb);

  // Construct Theora image buffer
  th_ycbcr_buffer ycbcr_buffer;
  cvToTheoraPlane(y,  ycbcr_buffer[0]);
  cvToTheoraPlane(cb, ycbcr_buffer[1]);
  cvToTheoraPlane(cr, ycbcr_buffer[2]);

  // Submit frame to the encoder
  int rval = th_encode_ycbcr_in(encoding_context_.get(), ycbcr_buffer);
  if (rval == TH_EFAULT) {
    ROS_ERROR("[theora] EFAULT in submitting uncompressed frame to encoder");
    return;
  }
  if (rval == TH_EINVAL) {
    ROS_ERROR("[theora] EINVAL in submitting uncompressed frame to encoder");
    return;
  }

  // Retrieve and publish encoded video data packets
  ogg_packet oggpacket;
  theora_image_transport::Packet output;
  while ((rval = th_encode_packetout(encoding_context_.get(), 0, &oggpacket)) > 0) {
    oggPacketToMsg(message.header, oggpacket, output);
    publish_fn(output);
  }
  if (rval == TH_EFAULT)
    ROS_ERROR("[theora] EFAULT in retrieving encoded video data packets");
}

void freeContext(th_enc_ctx* context)
{
  if (context) th_encode_free(context);
}

bool TheoraPublisher::ensureEncodingContext(const sensor_msgs::Image& image, const PublishFn& publish_fn) const
{
  /// @todo Check if encoding has changed
  if (encoding_context_ && encoder_setup_.pic_width == image.width &&
      encoder_setup_.pic_height == image.height)
    return true;

  // Theora has a divisible-by-sixteen restriction for the encoded frame size, so
  // scale the picture size up to the nearest multiple of 16 and calculate offsets.
  encoder_setup_.frame_width = (image.width + 15) & ~0xF;
  encoder_setup_.frame_height = (image.height + 15) & ~0xF;
  encoder_setup_.pic_width = image.width;
  encoder_setup_.pic_height = image.height;

  // Allocate encoding context. Smart pointer ensures that th_encode_free gets called.
  encoding_context_.reset(th_encode_alloc(&encoder_setup_), freeContext);
  if (!encoding_context_) {
    ROS_ERROR("[theora] Failed to create encoding context");
    return false;
  }

  updateKeyframeFrequency();

  th_comment comment;
  th_comment_init(&comment);
  boost::shared_ptr<th_comment> clear_guard(&comment, th_comment_clear);
  /// @todo Store image encoding in comment
  comment.vendor = strdup("Willow Garage theora_image_transport");

  // Construct the header and stream it in case anyone is already listening
  /// @todo Try not to send headers twice to some listeners
  stream_header_.clear();
  ogg_packet oggpacket;
  while (th_encode_flushheader(encoding_context_.get(), &comment, &oggpacket) > 0) {
    stream_header_.push_back(theora_image_transport::Packet());
    oggPacketToMsg(image.header, oggpacket, stream_header_.back());
    publish_fn(stream_header_.back());
  }
  return true;
}

void TheoraPublisher::oggPacketToMsg(const std_msgs::Header& header, const ogg_packet &oggpacket,
                                     theora_image_transport::Packet &msg) const
{
  msg.header     = header;
  msg.b_o_s      = oggpacket.b_o_s;
  msg.e_o_s      = oggpacket.e_o_s;
  msg.granulepos = oggpacket.granulepos;
  msg.packetno   = oggpacket.packetno;
  msg.data.resize(oggpacket.bytes);
  memcpy(&msg.data[0], oggpacket.packet, oggpacket.bytes);
}

void TheoraPublisher::updateKeyframeFrequency() const
{
  ogg_uint32_t desired_frequency = keyframe_frequency_;
  if (th_encode_ctl(encoding_context_.get(), TH_ENCCTL_SET_KEYFRAME_FREQUENCY_FORCE,
                    &keyframe_frequency_, sizeof(ogg_uint32_t)))
    ROS_ERROR("Failed to change keyframe frequency");
  if (keyframe_frequency_ != desired_frequency)
    ROS_WARN("Couldn't set keyframe frequency to %d, actually set to %d",
             desired_frequency, keyframe_frequency_);
}

} //namespace theora_image_transport
