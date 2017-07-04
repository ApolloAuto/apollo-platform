/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, The Apollo Authors.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

#ifndef SHAREDMEM_TRANSPORT_SHAREDMEM_PUBLISHER_IMPL_H
#define SHAREDMEM_TRANSPORT_SHAREDMEM_PUBLISHER_IMPL_H

#include "roscpp/SharedMemoryRegisterSegment.h"
#include "sharedmem_transport/sharedmem_util.h"
#include "ros/serialization.h"

namespace sharedmem_transport
{

/**
 * \brief Class for SharedMemoryPublisherImpl, which implements basic functions
 * for SharedMemoryPublisher.
 *
 */
class SharedMemoryPublisherImpl
{
public:
  SharedMemoryPublisherImpl();

  virtual ~SharedMemoryPublisherImpl();

  /**
   * \brief Create topic segment when pulish first msg
   *
   * @param topic: topic name
   * @param index: index which publish_fn needs
   * @param queue_size: block num
   * @param msg_size: msg size
   * @param real_alloc_size: alloc size
   * @param datatype: datatype
   * @param md5sum: md5sum
   * @param msg_def: msg_def
   * Return result for create topic segment, true or false;
   */
  bool create_topic_segement(const std::string& topic, int32_t& index, uint32_t queue_size, 
    uint64_t msg_size, uint64_t& real_alloc_size, std::string datatype,
    std::string md5sum, std::string msg_def);

  /**
   * \brief Publish msg
   *
   * @param message: message which will be published
   * @param queue_size: block num
   * @param sub_num: num of subscribers which needs to read the block
   * @param first_run: reallocated flag
   */
  void publish_msg(const ros::SerializedMessage& message, uint32_t queue_size, bool& first_run)
  {
    // Publish msg to shared memory
    int32_t last_index = 0;
    if (!_segment_mgr->write_data(message, queue_size, _descriptors_pub, _addr_pub, last_index))
    {
      ROS_WARN("Publish message failed");
      if (last_index == ROS_SHM_SEGMENT_WROTE_NUM)
      {
        first_run = true;
      }
    }
  }

private:
   /**
   * \brief Check if size ratio is valid
   *
   * Return result for size ratio, true or false
   */
  bool check_size_ratio();

  boost::interprocess::managed_shared_memory* _segment;
  SharedMemorySegment* _segment_mgr;
  SharedMemoryBlock* _descriptors_pub;
  uint8_t** _addr_pub;
};

} // namespace sharedmem_transport

#endif // SHAREDMEM_TRANSPORT_SHAREDMEM_PUBLISHER_IMPL_H
