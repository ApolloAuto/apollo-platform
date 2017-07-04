/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#include <nodelet/detail/callback_queue.h>
#include <nodelet/detail/callback_queue_manager.h>

#include <ros/callback_queue.h>

namespace nodelet
{
namespace detail
{

CallbackQueue::CallbackQueue(CallbackQueueManager* parent,
                             const ros::VoidConstPtr& tracked_object)
: parent_(parent)
, tracked_object_(tracked_object)
, has_tracked_object_(tracked_object)
{
}

CallbackQueue::~CallbackQueue()
{
}

void CallbackQueue::addCallback(const ros::CallbackInterfacePtr& cb, uint64_t owner_id)
{
  if (queue_.isEnabled())
  {
    queue_.addCallback(cb, owner_id);
    parent_->callbackAdded(shared_from_this());
  }
}

void CallbackQueue::removeByID(uint64_t owner_id)
{
  queue_.removeByID(owner_id);
}

uint32_t CallbackQueue::callOne()
{
  // Don't try to call the callback after its nodelet has been destroyed!
  ros::VoidConstPtr tracker;
  if (has_tracked_object_)
  {
    tracker = tracked_object_.lock();

    if (!tracker)
      return ros::CallbackQueue::Disabled;
  }
  
  return queue_.callOne();
}

} // namespace detail
} // namespace nodelet
