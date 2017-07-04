/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/publisher.h"
#include "ros/publication.h"
#include "ros/node_handle.h"
#include "ros/topic_manager.h"  
#include "ros/config_comm.h" 
#include "ros/this_node.h"   
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

namespace ros
{

Publisher::Impl::Impl() : unadvertised_(false), first_run_(true), default_transport_(SHARED_MEMORY) { }

Publisher::Impl::~Impl()
{
  ROS_DEBUG("Publisher on '%s' deregistering callbacks.", topic_.c_str());
  unadvertise();
}

bool Publisher::Impl::isValid() const
{
  return !unadvertised_;
}

void Publisher::Impl::unadvertise()
{
  if (!unadvertised_)
  {
    unadvertised_ = true;
    TopicManager::instance()->unadvertise(topic_, callbacks_);
    node_handle_.reset();
  }
}

Publisher::Publisher(const std::string& topic, const std::string& md5sum, 
  const std::string& datatype, const std::string& message_definition, const NodeHandle& node_handle, 
  const SubscriberCallbacksPtr& callbacks, const uint32_t& queue_size)
: impl_(boost::make_shared<Impl>())
{
  impl_->topic_ = topic;
  impl_->queue_size_ = queue_size;
  impl_->alloc_size_ = 0;
  impl_->md5sum_ = md5sum;
  impl_->datatype_ = datatype;
  impl_->message_definition_ = message_definition;
  impl_->node_handle_ = boost::make_shared<NodeHandle>(node_handle);
  impl_->callbacks_ = callbacks;
}

Publisher::Publisher(const Publisher& rhs)
{
  impl_ = rhs.impl_;
}

Publisher::~Publisher()
{
}

boost::interprocess::interprocess_mutex shm_pub_mutex_;

extern struct ConfigComm g_config_comm ;

void Publisher::publish(const boost::function<SerializedMessage(void)>& serfunc,
                        SerializedMessage& m) const
{
  if (!impl_)
  {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher (topic [%s])", impl_->topic_.c_str());
    return;
  }

  if (!impl_->isValid())
  {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher (topic [%s])", impl_->topic_.c_str());
    return;
  }
  
  if (impl_ && impl_->isValid())
  {
    if (!g_config_comm.transport_mode && 
      g_config_comm.topic_white_list.find(impl_->topic_) == g_config_comm.topic_white_list.end())
    {
      PublicationPtr publication_ptr = TopicManager::instance()->lookupPublication(impl_->topic_);
      if (!publication_ptr)
      {
        return;
      }
      
      impl_->default_transport_ = publication_ptr->getSubscriberlinksTransport();        

      if (impl_->default_transport_ == SHARED_MEMORY)
      {
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(shm_pub_mutex_);
        ROS_DEBUG_STREAM("Publish Tranport: " << impl_->topic_ << ", SHARED_MEMORY");
        publishShm(serfunc, m, impl_->datatype_, impl_->md5sum_, impl_->message_definition_);
      } 
      else if (impl_->default_transport_ == SOCKET)
      {
        ROS_DEBUG_STREAM("Publish Tranport: " << impl_->topic_ << ", SOCKET");
        TopicManager::instance()->publish(impl_->topic_, serfunc, m);
      } 
      else if (impl_->default_transport_ == BOTH)
      {
        ROS_DEBUG_STREAM("Publish Tranport: " << impl_->topic_ << ", BOTH");
        TopicManager::instance()->publish(impl_->topic_, serfunc, m); 

        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(shm_pub_mutex_);
        publishShm(serfunc, m, impl_->datatype_, impl_->md5sum_, impl_->message_definition_);
      } 
      else
      {
        ROS_ERROR_STREAM("Publish Tranport: " << impl_->default_transport_ << ", UNKNOWN");
      }
    } 
    else
    {
      ROS_DEBUG_STREAM("Publish Tranport: " << impl_->topic_ << ", SOCKET");
      TopicManager::instance()->publish(impl_->topic_, serfunc, m);
    }
  }
}


void Publisher::publishShm(const boost::function<SerializedMessage(void)>& serfunc, 
  SerializedMessage& message, std::string datatype, std::string md5sum, std::string msg_def) const
{
  SerializedMessage m2 = serfunc();
  message.buf = m2.buf;
  message.num_bytes = m2.num_bytes;
  message.message_start = m2.message_start;

  // Change queue_size to suitable size
  uint32_t queue_size_real = 5;
  
  if (!impl_->first_run_ && message.num_bytes - 4 >= impl_->alloc_size_)
  {
    ROS_DEBUG_STREAM("Reallocated msg queue start");
    
    // Get segment handler to remove segment
    boost::shared_ptr<sharedmem_transport::SharedMemoryUtil> shm_util(new sharedmem_transport::SharedMemoryUtil());
    boost::interprocess::managed_shared_memory* segment = NULL;
    sharedmem_transport::SharedMemorySegment* segment_mgr = NULL;
    sharedmem_transport::SharedMemoryBlock* descriptors_sub = NULL;

    if (shm_util->init_sharedmem(impl_->topic_.c_str(), 
      segment, segment_mgr, descriptors_sub, queue_size_real)) 
    {
      // Set reallocated flag
      segment_mgr->set_wrote_num(sharedmem_transport::ROS_SHM_SEGMENT_WROTE_NUM);
      delete segment;

      // Remove segment
      bool status = shm_util->remove_segment(impl_->topic_.c_str());

      // Set flag to init shared memory
      if (status)
      {
        impl_->first_run_ = true;
      }
    }

    ROS_DEBUG_STREAM("Reallocated msg queue end");
  }

  if (impl_->first_run_)
  {
    // Define variables
    int32_t index = 0;
    uint64_t msg_size = message.num_bytes - 4 ;

    // Create topic segment
    if (!impl_->shared_impl_.create_topic_segement(impl_->topic_, index,
      queue_size_real, msg_size, impl_->alloc_size_, datatype, md5sum, msg_def))
    {
      ROS_FATAL("Create topic segment failed");
      return;
    }

    // Reset first_run_ flag
    impl_->first_run_ = false;
  }

  // Publishing SHM message
  impl_->shared_impl_.publish_msg(message, queue_size_real, impl_->first_run_);
}

void Publisher::incrementSequence() const
{
  if (impl_ && impl_->isValid())
  {
    TopicManager::instance()->incrementSequence(impl_->topic_);
  }
}

void Publisher::shutdown()
{
  if (impl_)
  {
    impl_->unadvertise();
    impl_.reset();
  }
}

std::string Publisher::getTopic() const
{
  if (impl_)
  {
    return impl_->topic_;
  }

  return std::string();
}

uint32_t Publisher::getNumSubscribers() const
{
  if (impl_ && impl_->isValid())
  {
    return TopicManager::instance()->getNumSubscribers(impl_->topic_);
  }

  return 0;
}

bool Publisher::isLatched() const {
  PublicationPtr publication_ptr;
  if (impl_ && impl_->isValid()) 
  {
    publication_ptr = TopicManager::instance()->lookupPublication(impl_->topic_);
  } 
  else 
  {
    ROS_ASSERT_MSG(false, "Call to isLatched() on an invalid Publisher");
    throw ros::Exception("Call to isLatched() on an invalid Publisher");
  }
  if (publication_ptr) 
  {
    return publication_ptr->isLatched();
  } 
  else 
  {
    ROS_ASSERT_MSG(false, "Call to isLatched() on an invalid Publisher");
    throw ros::Exception("Call to isLatched() on an invalid Publisher");
  }
}

} // namespace ros
