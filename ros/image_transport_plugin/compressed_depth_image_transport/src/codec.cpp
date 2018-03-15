/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012 Willow Garage.
*  Copyright (c) 2016 Google, Inc.
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

#include <limits>
#include <string>
#include <vector>

#include <opencv2/highgui/highgui.hpp>

#include "cv_bridge/cv_bridge.h"
#include "compressed_depth_image_transport/codec.h"
#include "compressed_depth_image_transport/compression_common.h"
#include "ros/ros.h"

// If OpenCV3
#ifndef CV_VERSION_EPOCH
#include <opencv2/imgcodecs.hpp>
#endif

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

// Encoding and decoding of compressed depth images.
namespace compressed_depth_image_transport
{

sensor_msgs::Image::Ptr decodeCompressedDepthImage(const sensor_msgs::CompressedImage& message)
{

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message.header;

  // Assign image encoding
  std::string image_encoding = message.format.substr(0, message.format.find(';'));
  cv_ptr->encoding = image_encoding;

  // Decode message data
  if (message.data.size() > sizeof(ConfigHeader))
  {

    // Read compression type from stream
    ConfigHeader compressionConfig;
    memcpy(&compressionConfig, &message.data[0], sizeof(compressionConfig));

    // Get compressed image data
    const std::vector<uint8_t> imageData(message.data.begin() + sizeof(compressionConfig), message.data.end());

    // Depth map decoding
    float depthQuantA, depthQuantB;

    // Read quantization parameters
    depthQuantA = compressionConfig.depthParam[0];
    depthQuantB = compressionConfig.depthParam[1];

    if (enc::bitDepth(image_encoding) == 32)
    {
      cv::Mat decompressed;
      try
      {
        // Decode image data
        decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
      }
      catch (cv::Exception& e)
      {
        ROS_ERROR("%s", e.what());
        return sensor_msgs::Image::Ptr();
      }

      size_t rows = decompressed.rows;
      size_t cols = decompressed.cols;

      if ((rows > 0) && (cols > 0))
      {
        cv_ptr->image = Mat(rows, cols, CV_32FC1);

        // Depth conversion
        MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(),
                            itDepthImg_end = cv_ptr->image.end<float>();
        MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                                          itInvDepthImg_end = decompressed.end<unsigned short>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
        {
          // check for NaN & max depth
          if (*itInvDepthImg)
          {
            *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
          }
          else
          {
            *itDepthImg = std::numeric_limits<float>::quiet_NaN();
          }
        }

        // Publish message to user callback
        return cv_ptr->toImageMsg();
      }
    }
    else
    {
      // Decode raw image
      try
      {
        cv_ptr->image = cv::imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);
      }
      catch (cv::Exception& e)
      {
        ROS_ERROR("%s", e.what());
        return sensor_msgs::Image::Ptr();
      }

      size_t rows = cv_ptr->image.rows;
      size_t cols = cv_ptr->image.cols;

      if ((rows > 0) && (cols > 0))
      {
        // Publish message to user callback
        return cv_ptr->toImageMsg();
      }
    }
  }
  return sensor_msgs::Image::Ptr();
}

sensor_msgs::CompressedImage::Ptr encodeCompressedDepthImage(
    const sensor_msgs::Image& message,
    double depth_max, double depth_quantization, int png_level)
{

  // Compressed image message
  sensor_msgs::CompressedImage::Ptr compressed(new sensor_msgs::CompressedImage());
  compressed->header = message.header;
  compressed->format = message.encoding;

  // Compression settings
  std::vector<int> params;
  params.resize(3, 0);

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(message.encoding);
  int numChannels = enc::numChannels(message.encoding);

  // Image compression configuration
  ConfigHeader compressionConfig;
  compressionConfig.format = INV_DEPTH;

  // Compressed image data
  std::vector<uint8_t> compressedImage;

  // Update ros message format header
  compressed->format += "; compressedDepth";

  // Check input format
  params[0] = cv::IMWRITE_PNG_COMPRESSION;
  params[1] = png_level;

  if ((bitDepth == 32) && (numChannels == 1))
  {
    float depthZ0 = depth_quantization;
    float depthMax = depth_max;

    // OpenCV-ROS bridge
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(message);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("%s", e.what());
      return sensor_msgs::CompressedImage::Ptr();
    }

    const Mat& depthImg = cv_ptr->image;
    size_t rows = depthImg.rows;
    size_t cols = depthImg.cols;

    if ((rows > 0) && (cols > 0))
    {
      // Allocate matrix for inverse depth (disparity) coding
      Mat invDepthImg(rows, cols, CV_16UC1);

      // Inverse depth quantization parameters
      float depthQuantA = depthZ0 * (depthZ0 + 1.0f);
      float depthQuantB = 1.0f - depthQuantA / depthMax;

      // Matrix iterators
      MatConstIterator_<float> itDepthImg = depthImg.begin<float>(),
                               itDepthImg_end = depthImg.end<float>();
      MatIterator_<unsigned short> itInvDepthImg = invDepthImg.begin<unsigned short>(),
                                   itInvDepthImg_end = invDepthImg.end<unsigned short>();

      // Quantization
      for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
      {
        // check for NaN & max depth
        if (*itDepthImg < depthMax)
        {
          *itInvDepthImg = depthQuantA / *itDepthImg + depthQuantB;
        }
        else
        {
          *itInvDepthImg = 0;
        }
      }

      // Add coding parameters to header
      compressionConfig.depthParam[0] = depthQuantA;
      compressionConfig.depthParam[1] = depthQuantB;

      try
      {
        // Compress quantized disparity image
        if (cv::imencode(".png", invDepthImg, compressedImage, params))
        {
          float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
              / (float)compressedImage.size();
          ROS_DEBUG("Compressed Depth Image Transport - Compression: 1:%.2f (%lu bytes)", cRatio, compressedImage.size());
        }
        else
        {
          ROS_ERROR("cv::imencode (png) failed on input image");
          return sensor_msgs::CompressedImage::Ptr();
        }
      }
      catch (cv::Exception& e)
      {
        ROS_ERROR("%s", e.msg.c_str());
        return sensor_msgs::CompressedImage::Ptr();
      }
    }
  }
  // Raw depth map compression
  else if ((bitDepth == 16) && (numChannels == 1))
  {
    // OpenCV-ROS bridge
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(message);
    }
    catch (Exception& e)
    {
      ROS_ERROR("%s", e.msg.c_str());
      return sensor_msgs::CompressedImage::Ptr();
    }

    const Mat& depthImg = cv_ptr->image;
    size_t rows = depthImg.rows;
    size_t cols = depthImg.cols;

    if ((rows > 0) && (cols > 0))
    {
      unsigned short depthMaxUShort = static_cast<unsigned short>(depth_max * 1000.0f);

      // Matrix iterators
      MatIterator_<unsigned short> itDepthImg = cv_ptr->image.begin<unsigned short>(),
                                    itDepthImg_end = cv_ptr->image.end<unsigned short>();

      // Max depth filter
      for (; itDepthImg != itDepthImg_end; ++itDepthImg)
      {
        if (*itDepthImg > depthMaxUShort)
          *itDepthImg = 0;
      }

      // Compress raw depth image
      if (cv::imencode(".png", cv_ptr->image, compressedImage, params))
      {
        float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
            / (float)compressedImage.size();
        ROS_DEBUG("Compressed Depth Image Transport - Compression: 1:%.2f (%lu bytes)", cRatio, compressedImage.size());
      }
      else
      {
        ROS_ERROR("cv::imencode (png) failed on input image");
        return sensor_msgs::CompressedImage::Ptr();
      }
    }
  }
  else
  {
    ROS_ERROR("Compressed Depth Image Transport - Compression requires single-channel 32bit-floating point or 16bit raw depth images (input format is: %s).", message.encoding.c_str());
    return sensor_msgs::CompressedImage::Ptr();
  }

  if (compressedImage.size() > 0)
  {
    // Add configuration to binary output
    compressed->data.resize(sizeof(ConfigHeader));
    memcpy(&compressed->data[0], &compressionConfig, sizeof(ConfigHeader));

    // Add compressed binary data to messages
    compressed->data.insert(compressed->data.end(), compressedImage.begin(), compressedImage.end());

    return compressed;
  }

  return sensor_msgs::CompressedImage::Ptr();
}

}  // namespace compressed_depth_image_transport
