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

#ifndef SHAREDMEM_TRANSPORT_SHAREDMEM_BLOCK_H
#define SHAREDMEM_TRANSPORT_SHAREDMEM_BLOCK_H

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>

#include "ros/message.h"
#include "ros/serialization.h"
#include "ros/init.h"
#include <time.h>
#include <signal.h>
#include <setjmp.h>
#include <inttypes.h>

#include "ros/forwards.h"
#include "ros/message_deserializer.h"
#include "ros/subscription_callback_helper.h"
#include "ros/config_comm.h"

namespace sharedmem_transport
{
const float ROS_SHM_BLOCK_SIZE_RATIO = 1.5; // Allocated size compared to msg size for each block, default 1.5
const int ROS_SHM_UTIL_SEGMENT_NAME_SIZE = 1000; // Util segment name temp size
const int32_t ROS_SHM_SEGMENT_WROTE_NUM = -2; // Reallocated flag
const uint64_t ROS_SHM_BLOCK_MAX_SIZE = 12000000; // Block max size, Bit
const uint64_t ROS_SHM_BLOCK_SIZE = 2000000; // Block size, Bit
const uint64_t ROS_SHM_SEGMENT_SIZE = 3000000; // Segment size, Bit
const uint32_t ROS_SHM_BLOCK_MUTEX_TIMEOUT_SEC = 1; // Timeout for block mutex, sec

/**
 * \brief Class for Shared Memory Block, which is defined to describer the block.
 *
 */
class SharedMemoryBlock
{
public:
  SharedMemoryBlock();

  SharedMemoryBlock(bool wf, uint32_t rc, uint32_t ms, uint32_t as);

  virtual ~SharedMemoryBlock()
  {
  }

  /**
   * \brief Try reserve block for write
   *
   * Return result for try, true or false
   */
  bool try_reserve_for_radical_write();

  /**
   * \brief Try reserve block for read
   *
   * Return result for try, true or false
   */
  bool try_reserve_for_radical_read();

  /**
   * \brief Release reserve block for write
   *
   */
  void release_reserve_for_radical_write();

  /**
   * \brief Release reserve block for read
   *
   */
  void release_reserve_for_radical_read();

  /**
   * \brief Write to block
   *
   * @param dest: block address
   * @param msg: msg waited to be wrote
   * Return write result, true or false
   */
  bool write_to_block(uint8_t* dest, const ros::SerializedMessage& msg);

    /**
     * \brief Read from block
     *
     * @param src: block address
     * @param msg: msg waited to be deserialized to
     * @param helper: subscription callback helper
     * @param header_ptr: header pointer
     * Return read result, true or false
     */
  bool read_from_block(uint8_t* src, ros::VoidConstPtr& msg,
      ros::SubscriptionCallbackHelperPtr& helper, ros::M_stringPtr& header_ptr);

  /**
   * \brief Get real alloc_size
   *
   * @param real_alloc_size: alloc size which block is allocated
   */
  inline void get_alloc_size(uint64_t& real_alloc_size)
  {
    real_alloc_size = _alloc_size;
  }

private:
  /**
   * \brief Serialize msg to block
   *
   * @param dest: block address
   * @param msg: msg waited to be serialized
   * Return serialize result, true or false
   */
  inline bool serialize(uint8_t* dest, const ros::SerializedMessage& msg)
  {
    ROS_DEBUG("Serialize start!!!");

    {
      // Check size msg, before serialize
      _msg_size = msg.num_bytes - 4;

      if (_msg_size >= _alloc_size)
      {
        ROS_WARN("Msg size overflows the block, serialize failed");
        return false;
      }

      // Serialize msg to block
      ROS_DEBUG("Serialising to %p, %" PRIu64 " Bit", dest, _msg_size);
      memcpy(dest, msg.message_start, _msg_size);
    }

    ROS_DEBUG("Serialize end!!!");

    return true;
  }

private:
  // Mutex to protect access to _writing_flag, _reading_count
  boost::interprocess::interprocess_mutex _write_read_mutex;
  // When use conservative mechanism, publisher notified subscribers after writing msg
  boost::interprocess::interprocess_condition _read_cond;
  bool _writing_flag;
  uint32_t _reading_count;

  uint64_t _msg_size;
  uint64_t _alloc_size;
};

} // namespace sharedmem_transport
#endif // SHAREDMEM_TRANSPORT_SHAREDMEM_BLOCK_H
