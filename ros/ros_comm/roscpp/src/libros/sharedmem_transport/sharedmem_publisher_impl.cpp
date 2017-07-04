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

#include "sharedmem_transport/sharedmem_publisher_impl.h"

#include "ros/service.h"
#include "ros/ros.h"

namespace sharedmem_transport
{

SharedMemoryPublisherImpl::SharedMemoryPublisherImpl() : 
  _segment(NULL), _addr_pub(NULL)
{
}

SharedMemoryPublisherImpl::~SharedMemoryPublisherImpl()
{
  // This just disconnect from the segment, any subscriber can still finish reading it
  if (_segment)
  {
    delete _segment;
  }

  if (_addr_pub)
  {
    delete []_addr_pub;
  }
}

bool SharedMemoryPublisherImpl::create_topic_segement(
  const std::string& topic, int32_t& index, uint32_t queue_size, uint64_t msg_size, 
  uint64_t& real_alloc_size, std::string datatype, std::string md5sum, std::string msg_def)
{
  // Check if size_ratio is valid
  if (!check_size_ratio())
  {
    ROS_FATAL("Size ratio is invalid");
    return false;
  }

  // Get segment size
  uint64_t segment_size = ROS_SHM_BLOCK_SIZE_RATIO * queue_size * (msg_size + ROS_SHM_BLOCK_SIZE)
    + ROS_SHM_SEGMENT_SIZE;

  if (topic.length() >= ROS_SHM_UTIL_SEGMENT_NAME_SIZE)
  {
    ROS_ERROR("Topic name length overflow");
    return false;
  }
  
  ROS_DEBUG("segment_size %ld, queue_size %d, msg_size %ld",
    segment_size, queue_size, msg_size);

  SharedMemoryUtil sharedmem_util;

  // Create segment
  _segment = sharedmem_util.create_segment(topic.c_str(), segment_size);

  // Create segment mgr
  _segment_mgr = sharedmem_util.create_segment_mgr(_segment);
  ROS_DEBUG("Created segment %p, segment mgr %p", _segment, _segment_mgr);

  // Init all blocks from publisher
  _addr_pub = new uint8_t*[queue_size];

  if (!_segment || !_segment_mgr || !_addr_pub)
  {
    ROS_ERROR("Connected to SHM failed");
    delete _segment;
    _segment = NULL;
    index = -1;
    if (_addr_pub)
    {
      delete []_addr_pub;
      _addr_pub = NULL;
    }
    return false;
  }

  try
  {
    sharedmem_util.set_datatype(_segment, datatype);
    sharedmem_util.set_md5sum(_segment, md5sum);
    sharedmem_util.set_msg_def(_segment, msg_def);
  } catch (boost::interprocess::interprocess_exception& e)
  {
    ROS_ERROR("SHM save field failed");
  }

  return _segment_mgr->init_all_blocks(*_segment, queue_size, msg_size, real_alloc_size,
    _descriptors_pub, _addr_pub);
}

bool SharedMemoryPublisherImpl::check_size_ratio()
{
  if (ROS_SHM_BLOCK_SIZE_RATIO < 1.0)
  {
    ROS_ERROR("ROS_SHM_BLOCK_SIZE_RATIO %f is invalid, suggest [1.0, 3.0]",
      ROS_SHM_BLOCK_SIZE_RATIO);
    return false;
  }

  if (ROS_SHM_BLOCK_SIZE_RATIO > 3.0)
  {
    ROS_WARN("ROS_SHM_BLOCK_SIZE_RATIO %f is invalid, suggest [1.0, 3.0]",
      ROS_SHM_BLOCK_SIZE_RATIO);
  }

  return true;
}

} // namespace sharedmem_transport
