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

#include "sharedmem_transport/sharedmem_segment.h"

namespace sharedmem_transport
{
/**************************************************************************************
 * Public function in SharedMemorySegment
 **************************************************************************************/
bool SharedMemorySegment::init_all_blocks(boost::interprocess::managed_shared_memory& segment,
  uint32_t queue_size, uint64_t msg_size, uint64_t& real_alloc_size, SharedMemoryBlock*& descriptors_pub,
  uint8_t** addr_pub)
{
  // Lock mutex in segment
  boost::interprocess::scoped_lock < boost::interprocess::interprocess_mutex > lock(
    _wrote_num_mutex);

  // Init descriptors_pub for all blocks
  uint64_t alloc_size = (uint64_t)((msg_size + ROS_SHM_BLOCK_SIZE) * ROS_SHM_BLOCK_SIZE_RATIO);

  try
  {
    descriptors_pub = segment.find_or_construct < SharedMemoryBlock > ("BlockDescriptor")
      [queue_size](false, 0, msg_size, alloc_size);
  } catch (boost::interprocess::bad_alloc e)
  {
    ROS_ERROR("Block allocate error");
    return false;
  }

  if (!descriptors_pub)
  {
    ROS_ERROR("Not create descriptors_pub!");
    return false;
  }

  // Init all blocks
  for (uint32_t i = 0; i < queue_size; ++i)
  {
    std::stringstream block_name;
    block_name << i;
    
    uint8_t* addr = NULL;
    try
    {
      addr = segment.find_or_construct < uint8_t >
      (block_name.str().c_str())[alloc_size](0);
    } catch (boost::interprocess::bad_alloc e)
    {
      ROS_ERROR("Addr pub %d allocate error", i);
      return false;
    }

    if (!addr)
    {
      ROS_ERROR("Not create %d block!", i);
      return false;
    }

    addr_pub[i] = addr;
    ROS_DEBUG("pub init: connected block %d to index %d, addr %p", i, i, addr);
  }

  // Get real_alloc_size
  descriptors_pub->get_alloc_size(real_alloc_size);

  return true;
}

bool SharedMemorySegment::map_all_blocks(boost::interprocess::managed_shared_memory*& segment,
  uint32_t queue_size, uint8_t** addr_sub)
{
  // Lock mutex in segment
  boost::interprocess::scoped_lock < boost::interprocess::interprocess_mutex > lock(
    _wrote_num_mutex);

  // Map all blocks
  for (uint32_t i = 0; i < queue_size; ++i)
  {
    std::stringstream block_name;
    block_name << i;
    
    uint8_t* addr = NULL;
    try
    {
      addr = segment->find < uint8_t > (block_name.str().c_str()).first;
    } catch (boost::interprocess::bad_alloc e)
    {
      ROS_ERROR("Addr sub %d allocate error", i);
      return false;
    }

    if (!addr)
    {
      ROS_INFO("Not find %d block!", i);
      return false;
    }

    addr_sub[i] = addr;
    ROS_DEBUG("sub map: connected block %d to index %d, addr %p", i, i, addr);
  }

  return true;
}

bool SharedMemorySegment::write_data(const ros::SerializedMessage& msg,
  uint32_t queue_size,SharedMemoryBlock* descriptors_pub, uint8_t** addr_pub, int32_t& last_index)
{
  ROS_DEBUG("Write radical start!");
  int32_t block_index;

  {
    // Lock _wrote_num_mutex in segment
    ROS_DEBUG("Lock _wrote_num_mutex in segment");
    boost::interprocess::scoped_lock < boost::interprocess::interprocess_mutex >
      segment_lock(_wrote_num_mutex);

    // Reserve next writable block
    block_index = reserve_radical_writable_block(queue_size, descriptors_pub);
  }

  // Block needs to be reallocated
  if (block_index == ROS_SHM_SEGMENT_WROTE_NUM)
  {
    last_index = block_index;
    return false;
  }

  // Get descriptor current pointer
  SharedMemoryBlock* descriptors_curr = descriptors_pub + block_index;

  // Write to block
  bool result = descriptors_curr->write_to_block(addr_pub[block_index], msg);

  // Release reserve block, after we have wrote to block
  descriptors_curr->release_reserve_for_radical_write();

  // Check write result, if failed, return; if succeed, continue
  if (!result)
  {
    return false;
  }

  // Set _wrote_num to current
  set_wrote_num(block_index);

  // Publisher wrote done, notify subscriber read
  _wrote_num_cond.notify_all();

  ROS_DEBUG("Write radical end! %d", block_index);

  return true;
}

bool SharedMemorySegment::read_data(ros::VoidConstPtr& msg, int32_t& last_read_index,
  SharedMemoryBlock* descriptors_sub, uint8_t** addr_sub,
  ros::SubscriptionCallbackHelperPtr& helper, const std::string& topic,
  ros::M_stringPtr& header_ptr)
{
  ROS_DEBUG("Read radical start!");

  int32_t block_index;

  {
    // Lock _wrote_num_mutex in segment
    ROS_DEBUG("Lock _wrote_num_mutex in segment");
    boost::interprocess::scoped_lock < boost::interprocess::interprocess_mutex >
      segment_lock(_wrote_num_mutex);

    // Block needs to be reallocated
    if (_wrote_num == ROS_SHM_SEGMENT_WROTE_NUM)
    {
      last_read_index = _wrote_num;
      return false;
    }

    // Block is not available for read, or block has been read
    if (_wrote_num == -1 || _wrote_num == last_read_index)
    {
      ROS_DEBUG("Block %d is not available, or has been read", _wrote_num);

      // Define wait timeout
      boost::posix_time::ptime max_wait =
        boost::posix_time::microsec_clock::universal_time() +
        boost::posix_time::seconds(ROS_SHM_BLOCK_MUTEX_TIMEOUT_SEC);

      // Wait publisher wrote timeout
      if (!_wrote_num_cond.timed_wait(segment_lock, max_wait))
      {
        ROS_DEBUG("Wait radical publisher wrote topic %s, block %d timeout", 
          topic.c_str(), _wrote_num);
        return false;
      }
    }

    // Reserve next readable block failed
    if (!reserve_radical_readable_block(descriptors_sub))
    {
      return false;
    }

    // Reserve next readable block succeed
    block_index = _wrote_num;
  }

  // Get descriptor current pointer
  SharedMemoryBlock* descriptors_curr = descriptors_sub + block_index;

  // Read from block
  bool result = descriptors_curr->read_from_block(addr_sub[block_index], msg, helper, header_ptr);

  // Release reserve block, after we have read from block
  descriptors_curr->release_reserve_for_radical_read();

  // Check read result, if failed, return; if succeed, continue
  if (!result)
  {
    return false;
  }

  // Set last read num
  last_read_index = block_index;

  ROS_DEBUG("Read radical end! %d", block_index);

  return true;
}

/**************************************************************************************
 * Private function in SharedMemorySegment
 **************************************************************************************/
int32_t SharedMemorySegment::reserve_radical_writable_block(uint32_t queue_size,
  SharedMemoryBlock* descriptors_pub)
{
  // Get current _wrote_num
  int32_t wrote_curr = _wrote_num;

  // First publish
  if (wrote_curr == -1)
  {
    ++wrote_curr;
    return wrote_curr;
  }

  // Non-First publish
  while (wrote_curr != ROS_SHM_SEGMENT_WROTE_NUM && ros::ok())
  {
    ++wrote_curr;

    // Find in cycle
    if (wrote_curr >= (int32_t)queue_size)
    {
      wrote_curr = wrote_curr % queue_size;
    }

    if ((descriptors_pub + wrote_curr)->try_reserve_for_radical_write())
    {
      ROS_DEBUG("Reserve block %d succeed", wrote_curr);
      return wrote_curr;
    }
    else
    {
      ROS_DEBUG("Reserve block %d failed", wrote_curr);
    }
  }

  // ROS in not ok
  return wrote_curr;
}

bool SharedMemorySegment::reserve_radical_readable_block(SharedMemoryBlock* descriptors_sub)
{
  if ((descriptors_sub + _wrote_num)->try_reserve_for_radical_read())
  {
    ROS_DEBUG("Reserve block %d succeed", _wrote_num);
    return true;
  }
  else
  {
    ROS_DEBUG("Reserve block %d failed", _wrote_num);
    return false;
  }
}

} // namespace sharedmem_transport
