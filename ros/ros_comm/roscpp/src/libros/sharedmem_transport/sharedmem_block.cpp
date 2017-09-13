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
#include "sharedmem_transport/sharedmem_block.h"

namespace sharedmem_transport
{

/**************************************************************************************
 * Public function in SharedMemoryBlock
 **************************************************************************************/
SharedMemoryBlock::SharedMemoryBlock() :
  _writing_flag(false), 
  _reading_count(0), 
  _msg_size(0), 
  _alloc_size(0)
{
}

SharedMemoryBlock::SharedMemoryBlock(
  bool wf, uint32_t rc, uint32_t ms, uint32_t as) :
  _writing_flag(wf), 
  _reading_count(rc), 
  _msg_size(ms), 
  _alloc_size(as)
{
}

bool SharedMemoryBlock::try_reserve_for_radical_write()
{
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
    _write_read_mutex);

  // Block is not available for write
  if (_writing_flag || _reading_count)
  {
    return false;
  }

  // Block is available for write
  _writing_flag = true;
  return true;
}

bool SharedMemoryBlock::try_reserve_for_radical_read()
{
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
    _write_read_mutex);

  // Block is not available for read
  if (_writing_flag)
  {
      return false;
  }

  // Block is available for read
  ++_reading_count;
  return true;
}

void SharedMemoryBlock::release_reserve_for_radical_write()
{
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
    _write_read_mutex);

  _writing_flag = false;
}

void SharedMemoryBlock::release_reserve_for_radical_read()
{
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
    _write_read_mutex);

  --_reading_count;
}

bool SharedMemoryBlock::write_to_block(uint8_t* dest, const ros::SerializedMessage& msg)
{
  // Serialize Data
  bool result = serialize(dest, msg);

  return result;
}

bool SharedMemoryBlock::read_from_block(uint8_t* src, ros::VoidConstPtr& msg,
  ros::SubscriptionCallbackHelperPtr& helper, ros::M_stringPtr& header_ptr)
{
  // Deserialze msg from block
  ros::SubscriptionCallbackHelperDeserializeParams params ;
  params.buffer = src;
  params.length = (uint32_t)_msg_size;
  params.connection_header = header_ptr;
  msg = helper->deserialize(params);

  return true;
}

} // namespace sharedmem_transport
