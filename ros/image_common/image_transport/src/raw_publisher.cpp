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

#include <image_transport/raw_publisher.h>
#include <ros/static_assert.h>
#include <sensor_msgs/Image.h>

/** The code in the following namespace copies a lof of code from cv_bridge
 * It does not depend on cv_bridge to not depend on OpenCV
 * It re-defines a CvImage so that we can publish that object and not a
 * sensor_msgs::Image, which requires a memory copy
 */

class ImageTransportImage
{
public:
  sensor_msgs::Image image_; //!< ROS header
  const uint8_t* data_;           //!< Image data for use with OpenCV

  /**
   * \brief Empty constructor.
   */
  ImageTransportImage() {}

  /**
   * \brief Constructor.
   */
  ImageTransportImage(const sensor_msgs::Image& image, const uint8_t* data)
    : image_(image), data_(data)
  {
  }
};

/// @cond DOXYGEN_IGNORE
namespace ros {

namespace message_traits {

template<> struct MD5Sum<ImageTransportImage>
{
  static const char* value() { return MD5Sum<sensor_msgs::Image>::value(); }
  static const char* value(const ImageTransportImage&) { return value(); }

  static const uint64_t static_value1 = MD5Sum<sensor_msgs::Image>::static_value1;
  static const uint64_t static_value2 = MD5Sum<sensor_msgs::Image>::static_value2;
  
  // If the definition of sensor_msgs/Image changes, we'll get a compile error here.
  ROS_STATIC_ASSERT(MD5Sum<sensor_msgs::Image>::static_value1 == 0x060021388200f6f0ULL);
  ROS_STATIC_ASSERT(MD5Sum<sensor_msgs::Image>::static_value2 == 0xf447d0fcd9c64743ULL);
};

template<> struct DataType<ImageTransportImage>
{
  static const char* value() { return DataType<sensor_msgs::Image>::value(); }
  static const char* value(const ImageTransportImage&) { return value(); }
};

template<> struct Definition<ImageTransportImage>
{
  static const char* value() { return Definition<sensor_msgs::Image>::value(); }
  static const char* value(const ImageTransportImage&) { return value(); }
};

template<> struct HasHeader<ImageTransportImage> : TrueType {};

} // namespace ros::message_traits

namespace serialization {

template<> struct Serializer<ImageTransportImage>
{
  /// @todo Still ignoring endianness...
  
  template<typename Stream>
  inline static void write(Stream& stream, const ImageTransportImage& m)
  {
    stream.next(m.image_.header);
    stream.next((uint32_t)m.image_.height); // height
    stream.next((uint32_t)m.image_.width); // width
    stream.next(m.image_.encoding);
    uint8_t is_bigendian = 0;
    stream.next(is_bigendian);
    stream.next((uint32_t)m.image_.step);
    size_t data_size = m.image_.step*m.image_.height;
    stream.next((uint32_t)data_size);
    if (data_size > 0)
      memcpy(stream.advance(data_size), m.data_, data_size);
  }

  inline static uint32_t serializedLength(const ImageTransportImage& m)
  {
    size_t data_size = m.image_.step*m.image_.height;
    return serializationLength(m.image_.header) + serializationLength(m.image_.encoding) + 17 + data_size;
  }
};

} // namespace ros::serialization

} // namespace ros


namespace image_transport {

void RawPublisher::publish(const sensor_msgs::Image& message, const uint8_t* data) const
{
   getPublisher().publish(ImageTransportImage(message, data));
}

} //namespace image_transport
