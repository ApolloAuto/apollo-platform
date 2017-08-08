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

#include "sharedmem_transport/sharedmem_util.h"

namespace sharedmem_transport
{

/**************************************************************************************
 * Public function in SharedMemoryUtil
 **************************************************************************************/
bool SharedMemoryUtil::init_sharedmem(const char* topic_name,
  boost::interprocess::managed_shared_memory*& segment,
  sharedmem_transport::SharedMemorySegment*& segment_mgr,
  sharedmem_transport::SharedMemoryBlock*& descriptors,
  uint32_t& queue_size)
{
  char segment_name[ROS_SHM_UTIL_SEGMENT_NAME_SIZE];
  get_segment_name(topic_name, segment_name);

  try
  {
    segment = new boost::interprocess::managed_shared_memory(
      boost::interprocess::open_only, segment_name);
  } catch (boost::interprocess::interprocess_exception e)
  {
    return false;
  }

  if (!segment)
  {
    return false;
  }

  segment_mgr = (segment->find < SharedMemorySegment > ("Manager")).first;

  std::pair<SharedMemoryBlock*, std::size_t> descriptors_block(NULL, 0);

  descriptors_block = segment->find < SharedMemoryBlock > ("BlockDescriptor");
  descriptors = descriptors_block.first;
  queue_size = descriptors_block.second;

  if (!segment_mgr || !descriptors)
  {
    delete segment;
    return false;
  }

  return true;
}

boost::interprocess::managed_shared_memory* SharedMemoryUtil::get_segment(const char* topic_name)
{
  ROS_DEBUG("Get segment");

  char segment_name[ROS_SHM_UTIL_SEGMENT_NAME_SIZE];
  get_segment_name(topic_name, segment_name);

  ROS_DEBUG("get segment topic name: %s", topic_name);

  boost::interprocess::managed_shared_memory* segment = NULL;

  try
  {
    segment = new boost::interprocess::managed_shared_memory(
      boost::interprocess::open_only, segment_name);
  } catch (boost::interprocess::interprocess_exception e)
  {
    ROS_DEBUG("segment is null");
  }

  return segment;
}

boost::interprocess::managed_shared_memory* SharedMemoryUtil::create_segment(
  const char* topic_name, uint64_t segment_size)
{
  ROS_DEBUG("Create segment");

  char segment_name[ROS_SHM_UTIL_SEGMENT_NAME_SIZE];
  get_segment_name(topic_name, segment_name);

  boost::interprocess::managed_shared_memory* segment = NULL;

  try
  {
    segment = new boost::interprocess::managed_shared_memory(
      boost::interprocess::open_or_create, segment_name, segment_size);
  } catch (boost::interprocess::interprocess_exception e)
  {
    ROS_ERROR_STREAM("Create segment failed" << topic_name);
  }

  return segment;
}

SharedMemorySegment* SharedMemoryUtil::get_segment_mgr(
  boost::interprocess::managed_shared_memory*& segment)
{
  ROS_DEBUG("Connected to segment_mgr");

  SharedMemorySegment* segment_mgr = NULL;

  if (!segment)
  {
    ROS_ERROR("Segment is NULL");
    return segment_mgr;
  }

  segment_mgr = (segment->find < SharedMemorySegment > ("Manager")).first;

  return segment_mgr;
}

SharedMemorySegment* SharedMemoryUtil::create_segment_mgr(
  boost::interprocess::managed_shared_memory*& segment)
{
  ROS_DEBUG("Create segment_mgr");

  SharedMemorySegment* segment_mgr = NULL;

  if (!segment)
  {
    ROS_ERROR("Segment is NULL");
    return segment_mgr;
  }

  segment_mgr = segment->find_or_construct < SharedMemorySegment > ("Manager")();

  return segment_mgr;
}

bool SharedMemoryUtil::remove_segment(const char* topic_name)
{
  ROS_DEBUG("Remove segment %s", topic_name);

  char segment_name[ROS_SHM_UTIL_SEGMENT_NAME_SIZE];
  get_segment_name(topic_name, segment_name);
  bool status = false;

  try
  {
    status = boost::interprocess::shared_memory_object::remove(segment_name);
  } catch (boost::interprocess::interprocess_exception e)
  {
    ROS_ERROR_STREAM("Delete segment failed" << topic_name);
  }

  return status;
}

void SharedMemoryUtil::set_datatype(boost::interprocess::managed_shared_memory*& segment, std::string content)
{
  set_segment_string(segment, "datatype", content);
}

std::string SharedMemoryUtil::get_datatype(boost::interprocess::managed_shared_memory*& segment)
{
  return get_segment_string(segment, "datatype");
}

void SharedMemoryUtil::set_md5sum(boost::interprocess::managed_shared_memory*& segment, std::string content)
{
  set_segment_string(segment, "md5sum", content);
}

std::string SharedMemoryUtil::get_md5sum(boost::interprocess::managed_shared_memory*& segment)
{
  return get_segment_string(segment, "md5sum");
}

void SharedMemoryUtil::set_msg_def(boost::interprocess::managed_shared_memory*& segment, std::string content)
{
  set_segment_string(segment, "msg_def", content);
}

std::string SharedMemoryUtil::get_msg_def(boost::interprocess::managed_shared_memory*& segment)
{
  return get_segment_string(segment, "msg_def");
}

/**************************************************************************************
 * Private function in SharedMemoryUtil
 **************************************************************************************/
void SharedMemoryUtil::set_segment_string(boost::interprocess::managed_shared_memory*& segment, 
  std::string string_name, std::string string_content)
{
  segment->find_or_construct<SHMString>(string_name.c_str())(string_content.c_str(), segment->get_segment_manager());
}

std::string SharedMemoryUtil::get_segment_string(boost::interprocess::managed_shared_memory*& segment, 
  std::string string_name)
{
  std::pair<SHMString*, size_t> p = segment->find<SHMString>(string_name.c_str());
  if (!p.first)
  {
    return NULL;
  }
  return p.first->c_str();
}

void SharedMemoryUtil::get_segment_name(const char* topic_name, char* segment_name)
{
  if (strlen(topic_name) >= ROS_SHM_UTIL_SEGMENT_NAME_SIZE)
  {
    ROS_ERROR("Topic name length overflow");
    return;
  }
  snprintf(segment_name, ROS_SHM_UTIL_SEGMENT_NAME_SIZE, "%s", topic_name);
  replace_all(segment_name, '/', ':');
}

char* SharedMemoryUtil::replace_all(char* src, char old_char, char new_char)
{
  char* head = src;
  int size = strlen(src);

  for (int i = 0; i < size; ++i)
  {
    if (*src == old_char)
    {
      *src = new_char;
    }

    ++src;
  }

  return head;
}

} // namespace sharedmem_transport

extern "C"
bool remove_segment(const char* topic_name)
{
  sharedmem_transport::SharedMemoryUtil shm_util;
  return shm_util.remove_segment(topic_name);
} 

