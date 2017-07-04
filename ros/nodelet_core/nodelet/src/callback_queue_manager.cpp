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

#include <nodelet/detail/callback_queue_manager.h>
#include <nodelet/detail/callback_queue.h>
#include <ros/callback_queue.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/type_traits/alignment_of.hpp>
#include <boost/bind.hpp>

#include <ros/assert.h>

namespace nodelet
{
namespace detail
{

CallbackQueueManager::CallbackQueueManager(uint32_t num_worker_threads)
: running_(true),
  num_worker_threads_(num_worker_threads)
{
  if (num_worker_threads_ == 0)
    num_worker_threads_ = boost::thread::hardware_concurrency();
  
  tg_.create_thread(boost::bind(&CallbackQueueManager::managerThread, this));

  size_t num_threads = getNumWorkerThreads();
  thread_info_.reset( new ThreadInfo[num_threads] );
  for (size_t i = 0; i < num_threads; ++i)
  {
    tg_.create_thread(boost::bind(&CallbackQueueManager::workerThread, this, &thread_info_[i]));
  }
}

CallbackQueueManager::~CallbackQueueManager()
{
  stop();
  
#ifdef NODELET_QUEUE_DEBUG
  // Write out task assignment histories for each thread
  typedef ThreadInfo::Record Record;
  for (size_t i = 0; i < num_threads; ++i)
  {
    char filename[128];
    sprintf(filename, "thread_history_%d.txt", (int)i);
    FILE* file = fopen(filename, "w");
    fprintf(file, "# timestamps tasks threaded\n");
    const std::vector<Record>& history = thread_info_[i].history;
    for (int j = 0; j < (int)history.size(); ++j)
    {
      Record r = history[j];
      fprintf(file, "%.6f %u %d\n", r.stamp, r.tasks, (int)r.threaded);
    }
    fclose(file);
  }
#endif
}

void CallbackQueueManager::stop()
{
  running_ = false;
  {
    boost::mutex::scoped_lock lock(waiting_mutex_);
    waiting_cond_.notify_all();
  }

  size_t num_threads = getNumWorkerThreads();
  for (size_t i = 0; i < num_threads; ++i)
  {
    boost::mutex::scoped_lock lock(thread_info_[i].queue_mutex);
    thread_info_[i].queue_cond.notify_all();
  }

  tg_.join_all();
}

uint32_t CallbackQueueManager::getNumWorkerThreads()
{
  return num_worker_threads_;
}

void CallbackQueueManager::addQueue(const CallbackQueuePtr& queue, bool threaded)
{
  boost::mutex::scoped_lock lock(queues_mutex_);

  QueueInfoPtr& info = queues_[queue.get()];
  ROS_ASSERT(!info);
  info.reset(new QueueInfo);
  info->queue = queue;
  info->threaded = threaded;
}

void CallbackQueueManager::removeQueue(const CallbackQueuePtr& queue)
{
  boost::mutex::scoped_lock lock(queues_mutex_);
  ROS_ASSERT(queues_.find(queue.get()) != queues_.end());

  queues_.erase(queue.get());
}

void CallbackQueueManager::callbackAdded(const CallbackQueuePtr& queue)
{
  {
    boost::mutex::scoped_lock lock(waiting_mutex_);
    waiting_.push_back(queue);
  }

  waiting_cond_.notify_all();
}

CallbackQueueManager::ThreadInfo* CallbackQueueManager::getSmallestQueue()
{
  size_t smallest = std::numeric_limits<size_t>::max();
  uint32_t smallest_index = 0xffffffff;
  for (unsigned i = 0; i < num_worker_threads_; ++i)
  {
    ThreadInfo& ti = thread_info_[i];

    size_t size = ti.calling;
    if (size == 0)
    {
      return &ti;
    }

    if (size < smallest)
    {
      smallest = size;
      smallest_index = i;
    }
  }

  return &thread_info_[smallest_index];
}

void CallbackQueueManager::managerThread()
{
  V_Queue local_waiting;

  while (running_)
  {
    {
      boost::mutex::scoped_lock lock(waiting_mutex_);

      while (waiting_.empty() && running_)
      {
        waiting_cond_.wait(lock);
      }

      if (!running_)
      {
        return;
      }

      local_waiting.swap(waiting_);
    }

    {
      boost::mutex::scoped_lock lock(queues_mutex_);

      V_Queue::iterator it = local_waiting.begin();
      V_Queue::iterator end = local_waiting.end();
      for (; it != end; ++it)
      {
        CallbackQueuePtr& queue = *it;

        M_Queue::iterator it = queues_.find(queue.get());
        if (it != queues_.end())
        {
          QueueInfoPtr& info = it->second;
          ThreadInfo* ti = 0;
          if (info->threaded)
          {
            // If this queue is thread-safe we immediately add it to the thread with the least work queued
            ti = getSmallestQueue();
          }
          else
          {
            // If this queue is non-thread-safe and has no in-progress calls happening, we add it to the
            // thread with the least work queued.  If the queue already has calls in-progress we add it
            // to the thread it's already being called from
            boost::mutex::scoped_lock lock(info->st_mutex);

            if (info->in_thread == 0)
            {
              ti = getSmallestQueue();
              info->thread_index = ti - thread_info_.get();
            }
            else
            {
              ti = &thread_info_[info->thread_index];
            }

            ++info->in_thread;
          }

          {
            boost::mutex::scoped_lock lock(ti->queue_mutex);
            ti->queue.push_back(std::make_pair(queue, info));
            ++ti->calling;
#ifdef NODELET_QUEUE_DEBUG
            double stamp = ros::WallTime::now().toSec();
            uint32_t tasks = ti->calling;
            ti->history.push_back(ThreadInfo::Record(stamp, tasks, true));
#endif
          }

          ti->queue_cond.notify_all();
        }
      }
    }

    local_waiting.clear();
  }
}

void CallbackQueueManager::workerThread(ThreadInfo* info)
{
  std::vector<std::pair<CallbackQueuePtr, QueueInfoPtr> > local_queues;

  while (running_)
  {
    {
      boost::mutex::scoped_lock lock(info->queue_mutex);

      while (info->queue.empty() && running_)
      {
        info->queue_cond.wait(lock);
      }

      if (!running_)
      {
        return;
      }

      info->queue.swap(local_queues);
    }

    std::vector<std::pair<CallbackQueuePtr, QueueInfoPtr> >::iterator it = local_queues.begin();
    std::vector<std::pair<CallbackQueuePtr, QueueInfoPtr> >::iterator end = local_queues.end();
    for (; it != end; ++it)
    {
      CallbackQueuePtr& queue = it->first;
      QueueInfoPtr& qi = it->second;
      if (queue->callOne() == ros::CallbackQueue::TryAgain)
      {
        callbackAdded(queue);
      }
      --info->calling;

      if (!qi->threaded)
      {
        boost::mutex::scoped_lock lock(qi->st_mutex);
        --qi->in_thread;
      }
    }

    local_queues.clear();

  }
}

} // namespace detail
} // namespace nodelet
