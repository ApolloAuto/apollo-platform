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

#include "compressed_image_transport/compressed_publisher.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/make_shared.hpp>

#include "compressed_image_transport/compression_common.h"

#include <vector>
#include <sstream>

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

namespace compressed_image_transport
{

void CompressedPublisher::advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                                        const image_transport::SubscriberStatusCallback &user_connect_cb,
                                        const image_transport::SubscriberStatusCallback &user_disconnect_cb,
                                        const ros::VoidPtr &tracked_object, bool latch)
{
  typedef image_transport::SimplePublisherPlugin<sensor_msgs::CompressedImage> Base;
  Base::advertiseImpl(nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb, tracked_object, latch);

  // Set up reconfigure server for this topic
  reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
  ReconfigureServer::CallbackType f = boost::bind(&CompressedPublisher::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);
}

void CompressedPublisher::configCb(Config& config, uint32_t level)
{
  config_ = config;
}

void CompressedPublisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
{
  // Compressed image message
  sensor_msgs::CompressedImage compressed;
  compressed.header = message.header;
  compressed.format = message.encoding;

  // Compression settings
  std::vector<int> params;
  params.resize(3, 0);

  // Get codec configuration
  compressionFormat encodingFormat = UNDEFINED;
  if (config_.format == compressed_image_transport::CompressedPublisher_jpeg)
    encodingFormat = JPEG;
  if (config_.format == compressed_image_transport::CompressedPublisher_png)
    encodingFormat = PNG;

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(message.encoding);
  int numChannels = enc::numChannels(message.encoding);

  switch (encodingFormat)
  {
    // JPEG Compression
    case JPEG:
    {
      params[0] = CV_IMWRITE_JPEG_QUALITY;
      params[1] = config_.jpeg_quality;

      // Update ros message format header
      compressed.format += "; jpeg compressed ";

      // Check input format
      if ((bitDepth == 8) || (bitDepth == 16))
      {
        // Target image format
        std::string targetFormat;
        if (enc::isColor(message.encoding))
        {
          // convert color images to BGR8 format
          targetFormat = "bgr8";
          compressed.format += targetFormat;
        }

        // OpenCV-ros bridge
        try
        {
          boost::shared_ptr<CompressedPublisher> tracked_object;
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, tracked_object, targetFormat);

          // Compress image
          if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params))
          {

            float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
                / (float)compressed.data.size();
            ROS_DEBUG("Compressed Image Transport - Codec: jpg, Compression Ratio: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());
          }
          else
          {
            ROS_ERROR("cv::imencode (jpeg) failed on input image");
          }
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("%s", e.what());
        }
        catch (cv::Exception& e)
        {
          ROS_ERROR("%s", e.what());
        }

        // Publish message
        publish_fn(compressed);
      }
      else
        ROS_ERROR("Compressed Image Transport - JPEG compression requires 8/16-bit color format (input format is: %s)", message.encoding.c_str());

      break;
    }
      // PNG Compression
    case PNG:
    {
      params[0] = CV_IMWRITE_PNG_COMPRESSION;
      params[1] = config_.png_level;

      // Update ros message format header
      compressed.format += "; png compressed ";

      // Check input format
      if ((bitDepth == 8) || (bitDepth == 16))
      {

        // Target image format
        stringstream targetFormat;
        if (enc::isColor(message.encoding))
        {
          // convert color images to RGB domain
          targetFormat << "bgr" << bitDepth;
          compressed.format += targetFormat.str();
        }

        // OpenCV-ros bridge
        try
        {
          boost::shared_ptr<CompressedPublisher> tracked_object;
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, tracked_object, targetFormat.str());

          // Compress image
          if (cv::imencode(".png", cv_ptr->image, compressed.data, params))
          {

            float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
                / (float)compressed.data.size();
            ROS_DEBUG("Compressed Image Transport - Codec: png, Compression Ratio: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());
          }
          else
          {
            ROS_ERROR("cv::imencode (png) failed on input image");
          }
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("%s", e.what());
          return;
        }
        catch (cv::Exception& e)
        {
          ROS_ERROR("%s", e.what());
          return;
        }

        // Publish message
        publish_fn(compressed);
      }
      else
        ROS_ERROR("Compressed Image Transport - PNG compression requires 8/16-bit encoded color format (input format is: %s)", message.encoding.c_str());
      break;
    }

    default:
      ROS_ERROR("Unknown compression type '%s', valid options are 'jpeg' and 'png'", config_.format.c_str());
      break;
  }

}

} //namespace compressed_image_transport
