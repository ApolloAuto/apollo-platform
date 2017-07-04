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

#ifndef NODELET_CALLBACK_QUEUE_H
#define NODELET_CALLBACK_QUEUE_H

#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <boost/enable_shared_from_this.hpp>

namespace ros
{
class CallbackQueue;
}

namespace nodelet
{

namespace detail
{

class CallbackQueueManager;

class CallbackQueue : public ros::CallbackQueueInterface,
                      public boost::enable_shared_from_this<CallbackQueue>
{
public:
  CallbackQueue(CallbackQueueManager* parent,
                const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr());
  ~CallbackQueue();

  virtual void addCallback(const ros::CallbackInterfacePtr& callback, uint64_t owner_id = 0);
  virtual void removeByID(uint64_t owner_id);

  uint32_t callOne();

private:
  CallbackQueueManager* parent_;
  ros::CallbackQueue queue_;
  ros::VoidConstWPtr tracked_object_;
  bool has_tracked_object_;
};

} // namespace detail
} // namespace nodelet

#endif // NODELET_CALLBACK_QUEUE_H
