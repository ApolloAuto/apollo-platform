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

#ifndef SHAREDMEM_TRANSPORT_SHAREDMEM_SEGMENT_H
#define SHAREDMEM_TRANSPORT_SHAREDMEM_SEGMENT_H

#include "sharedmem_transport/sharedmem_block.h"
#include "ros/message_deserializer.h"
#include "ros/subscription_callback_helper.h"
#include "ros/forwards.h"
#include "ros/datatypes.h"

namespace sharedmem_transport
{

/**
 * \brief Class for Shared Memory Segment, which is defined to describer the segment
 *
 */
class SharedMemorySegment
{
public:
  SharedMemorySegment() : 
  _wrote_num(-1)
  {
  }

  virtual ~SharedMemorySegment()
  {
  }

  /**
   * \brief Init all blocks from publisher, before publisher write fisrt msg
   *
   * @param segment: segment pointer
   * @param queue_size: total block num which needs to be allocated
   * @param msg_size: single block size which needs to be allocated
   * @param real_alloc_size: single block size which has been allocated
   * @param descriptors_pub: descriptors maped address from sharedmem publisher
   * @param addr_pub: block maped address from sharedmem publisher
   * Return init result, true or false
   */
  bool init_all_blocks(boost::interprocess::managed_shared_memory& segment,
    uint32_t queue_size, uint64_t msg_size, uint64_t& real_alloc_size,
    SharedMemoryBlock*& descriptors_pub,
    uint8_t** addr_pub);

  /**
   * \brief Map all blocks from subscriber, before subscriber read first msg
   *
   * @param segment: segment pointer
   * @param queue_size: total block num
   * @param addr_sub: block maped address from sharedmem subscriber
   * Return map result, true or false
   */
  bool map_all_blocks(boost::interprocess::managed_shared_memory*& segment,
    uint32_t queue_size, uint8_t** addr_sub);

  /**
   * \brief Write data to block for sharedmem publisher
   *
   * @param msg: msg waited to be wrote
   * @param queue_size: total block num
   * @param descriptors_pub: descriptors maped address from sharedmem publisher
   * @param addr_pub: block maped address from sharedmem publisher
   * @param last_index: index which publisher wrote last
   * Return write result, true or false
   */
  bool write_data(const ros::SerializedMessage& msg, uint32_t queue_size,
    SharedMemoryBlock* descriptors_pub, uint8_t** addr_pub, int32_t& last_index);

  /**
     * \brief Read data from block for sharedmem subscriber
     *
     * @param msg: msg waited to be read
     * @param last_read_index: index which is used to record last readable block
     * @param descriptors_sub: descriptors maped address from sharedmem subscriber
     * @param addr_sub: block maped address from sharedmem subscriber
     * @param helper: subscription callback helper
     * @param topic: topic name
     * @param header_ptr: header pointer
     * Return read result, true or false
     */
  bool read_data(ros::VoidConstPtr& msg, int32_t& last_read_index,
    SharedMemoryBlock* descriptors_sub, uint8_t** addr_sub,
    ros::SubscriptionCallbackHelperPtr& helper, const std::string& topic,
    ros::M_stringPtr& header_ptr);

  /**
   * \brief Set _wrote_num, after publisher wrote data to block
   *
   * @param wrote_num: block index which has been wrote last
   */
  inline void set_wrote_num(int32_t wrote_num)
  {
    boost::interprocess::scoped_lock < boost::interprocess::interprocess_mutex > lock(
      _wrote_num_mutex);

    _wrote_num = wrote_num;
  }

  /**
   * \brief Get _wrote_num, when publisher or subscriber needs
   *
   * Return block index which has been wrote last
   */
  inline int32_t get_wrote_num()
  {
    boost::interprocess::scoped_lock < boost::interprocess::interprocess_mutex > lock(
      _wrote_num_mutex);

    return _wrote_num;
  }

private:
  /**
   * \brief Reserve writable block according _wrote_num
   *
   * @param queue_size: block num
   * @param descriptors_pub: descriptors maped address from sharedmem publisher
   * Return block index which is used to record writable block
   */
  int32_t reserve_radical_writable_block(uint32_t queue_size, 
    SharedMemoryBlock* descriptors_pub);

  /**
   * \brief Reserve readable block according _wrote_num
   *
   * @param descriptors_sub: descriptors maped address from sharedmem subscriber
   * Return result for reserve readable block, true or false
   */
  bool reserve_radical_readable_block(SharedMemoryBlock* descriptors_sub);

private:
  // Mutex to protect access _wrote_num
  boost::interprocess::interprocess_mutex _wrote_num_mutex;
  boost::interprocess::interprocess_condition _wrote_num_cond;
  int32_t _wrote_num;
};

} // namespace sharedmem_transport

#endif // SHAREDMEM_TRANSPORT_SHAREDMEM_SEGMENT_H
