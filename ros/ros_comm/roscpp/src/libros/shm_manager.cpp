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
#include "ros/shm_manager.h"

namespace ros {

ShmManagerPtr g_shm_manager;
std::mutex g_shm_manager_mutex;
extern struct ConfigComm g_config_comm;

const ShmManagerPtr& ShmManager::instance() 
{
  if (!g_shm_manager)
  {
    std::lock_guard<std::mutex> lg(g_shm_manager_mutex);

    if (!g_shm_manager)
    {
      g_shm_manager.reset(new ShmManager);
    }
  }
  return g_shm_manager;
}

ShmManager::ShmManager()
  :started_(false)
{
}

ShmManager::~ShmManager()
{
  if (started_)
  {
    shutdown();        
  }
}

void ShmManager::start()
{
  started_ = true;
  ROS_DEBUG("roscpp shm manager start!");
  server_thread_ = std::thread([this]()
  {
    this->threadFunc();
  });
}

void ShmManager::shutdown()
{
  if (!started_)
  {
    return;
  }
  started_ = false;

  for (std::vector <std::thread>::iterator th = shm_threads_.begin();
    th != shm_threads_.end(); ++th)
  {
    th->join();
  }
  shm_threads_.clear();

  server_thread_.join();

  for (std::map<std::string, ItemShm>::iterator sh = shm_map_.begin();
    sh != shm_map_.end(); ++sh)
  {
    if (sh->second.addr_sub)
    {
      delete [](sh->second.addr_sub);
    }
    
    if (sh->second.segment)
    {
      delete sh->second.segment;
    }

    if (sh->second.segment_mgr)
    {
      delete sh->second.segment_mgr;
    }
  }

  ROS_DEBUG("roscpp shm manager exit!");
}

void ShmManager::threadFunc()
{
  sharedmem_transport::SharedMemoryUtil sharedmem_util;
  std::string topic;

  while (started_ && ros::ok())
  {
    if (TopicManager::instance()->getNumSubscriptions() > 0)
    {
      L_Subscription subs = TopicManager::instance()->getAllSubscription();
      L_Subscription::iterator it;
      if (!g_config_comm.transport_mode && 
        shm_map_.size() < TopicManager::instance()->getNumSubscriptions())
      {
        for (it = subs.begin(); it != subs.end(); ++it) 
        {
          topic = ros::names::resolve((*it)->getName());

          if (g_config_comm.topic_white_list.find(topic) != 
            g_config_comm.topic_white_list.end() || 
            shm_map_.find(topic) != shm_map_.end() ||
            (*it)->get_publisher_links().size() == 0)
          {
            continue;
          }
                    
          boost::interprocess::managed_shared_memory* segment = NULL;
          sharedmem_transport::SharedMemorySegment* segment_mgr = NULL;
          sharedmem_transport::SharedMemoryBlock* descriptors_sub = NULL;
          uint32_t queue_size = 0;

          if (sharedmem_util.init_sharedmem(topic.c_str(), 
            segment, segment_mgr, descriptors_sub, queue_size)) 
          {    
            uint8_t** addr_sub = new uint8_t*[queue_size];

            if (!addr_sub)
            {
              ROS_DEBUG_STREAM("Init address for subscriber failed");
              continue;
            }
                        
            if (!segment_mgr->map_all_blocks(segment, queue_size, addr_sub)) 
            {
              ROS_DEBUG_STREAM("Map all blocks failed");
              if (addr_sub)
              {
                delete []addr_sub;
              }
              continue;
            }
            
            ItemShm item;
            item.segment = segment;
            item.segment_mgr = segment_mgr;
            item.descriptors_sub = descriptors_sub;
            item.queue_size = queue_size;
            item.addr_sub = addr_sub;
            item.shm_poll_flag = false;

            item.shm_sub_ptr = (*it);
            item.topic_name = (*it)->getName();
            item.callerid = this_node::getName();
            item.md5sum = sharedmem_util.get_md5sum(segment);
            item.datatype = sharedmem_util.get_datatype(segment);
            item.message_definition = sharedmem_util.get_msg_def(segment);

            boost::mutex::scoped_lock lock(shm_map_mutex_);

            shm_map_[topic] = item;
          }

          boost::mutex::scoped_lock lock(shm_first_msg_map_mutex_);

          if (shm_skip_first_msg_.find(topic) == shm_skip_first_msg_.end())
          {
            if (segment == NULL)
            {
              shm_skip_first_msg_[topic] = false;
            }
            else
            {
              shm_skip_first_msg_[topic] = true;
            }
          }
        }
      }

      if (shm_map_.size() != 0)
      {
        for (it = subs.begin(); it != subs.end(); ++it) 
        {
          topic = ros::names::resolve((*it)->getName());
          if (shm_map_.find(topic) != shm_map_.end() && !shm_map_[topic].shm_poll_flag)
          {
            // Set tag true, when we start thread for the topic 
            shm_map_[topic].shm_poll_flag = true;

            // Start thread for each topic
            shm_threads_.push_back(std::thread([&, topic]() 
            {
              bool exit = false;
              int32_t read_index = -1;
              bool is_new_msg = false;
              M_stringPtr header_ptr(new M_string());
              ros::VoidConstPtr msg;
              SerializedMessage m;

              (*header_ptr)["topic"] = shm_map_[topic].topic_name;
              (*header_ptr)["md5sum"] = shm_map_[topic].md5sum;
              (*header_ptr)["message_definition"] = shm_map_[topic].message_definition;
              (*header_ptr)["callerid"] = shm_map_[topic].callerid;
              (*header_ptr)["type"] = shm_map_[topic].datatype;

              while (!exit && started_ && ros::ok()) 
              {
                {
                  if (!((shm_map_[topic].shm_sub_ptr)->get_helper()))
                  {
                    continue;
                  }

                  is_new_msg = shm_map_[topic].segment_mgr->read_data(msg, read_index,
                    shm_map_[topic].descriptors_sub, shm_map_[topic].addr_sub,
                    (shm_map_[topic].shm_sub_ptr)->get_helper(), topic, header_ptr);

                  // Block needs to be allocated
                  if (read_index == sharedmem_transport::ROS_SHM_SEGMENT_WROTE_NUM ||
                    shm_map_[topic].shm_sub_ptr->get_publisher_links().size() == 0)
                  {
                    {
                      boost::mutex::scoped_lock lock(shm_map_mutex_);
                      
                      if (shm_map_[topic].addr_sub)
                      {
                        delete [](shm_map_[topic].addr_sub);
                      }
                      shm_map_.erase(topic);
                    }
                    
                    {
                      boost::mutex::scoped_lock lock(shm_first_msg_map_mutex_);
                      shm_skip_first_msg_.erase(topic);
                    }

                    is_new_msg = false;
                    exit = true;
                  }

                  // New message is coming
                  if (is_new_msg)
                  {
                    if (shm_skip_first_msg_[topic] == true)
                    {
                      // Skip first message
                      shm_skip_first_msg_[topic] = false;
                      continue;
                    }

                    m.message = msg;
                    m.type_info = &((shm_map_[topic].shm_sub_ptr)->get_helper()->getTypeInfo());
                    shm_map_[topic].shm_sub_ptr->handleMessage(m, false, true, header_ptr, NULL);
                  }
                }
              }
            }));
          }
        }
      }
    }
        
    // Sleep 10ms for next cycle
    usleep(10000);
  }
}

}

/* vim: set ts=4 sw=4 sts=4 tw=100 */
