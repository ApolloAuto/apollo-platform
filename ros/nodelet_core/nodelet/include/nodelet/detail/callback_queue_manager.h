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

#ifndef NODELET_CALLBACK_QUEUE_MANAGER_H
#define NODELET_CALLBACK_QUEUE_MANAGER_H

#include <ros/types.h>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <boost/unordered_map.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/detail/atomic_count.hpp>

#include <vector>
#include <deque>

namespace nodelet
{
namespace detail
{
class CallbackQueue;
typedef boost::shared_ptr<CallbackQueue> CallbackQueuePtr;

/**
 * \brief Internal use
 *
 * Manages a set of callback queues, potentially calling callbacks from them concurrently in
 * different threads.  Essentially a task manager specialized for callback queues.
 *
 * Uses 1 manager thread + N worker threads.  The manager thread gives work to the worker threads by
 * finding the thread with the fewest pending tasks and appending to that list.  This does mean that a
 * single long-running callback can potentially block other callbacks from being executed.  Some kind of
 * work-stealing could mitigate this, and is a good direction for future work.
 */
class CallbackQueueManager
{
public:
  /**
   * \brief Constructor
   *
   * By default, uses the number of hardware threads available on the current system.
   */
  CallbackQueueManager(uint32_t num_worker_threads = 0);
  ~CallbackQueueManager();

  void addQueue(const CallbackQueuePtr& queue, bool threaded);
  void removeQueue(const CallbackQueuePtr& queue);
  void callbackAdded(const CallbackQueuePtr& queue);

  uint32_t getNumWorkerThreads();

  void stop();

private:
  void managerThread();
  struct ThreadInfo;
  void workerThread(ThreadInfo*);

  class ThreadInfo;
  ThreadInfo* getSmallestQueue();

  struct QueueInfo
  {
    QueueInfo()
    : threaded(false)
    , thread_index(0xffffffff)
    , in_thread(0)
    {}

    CallbackQueuePtr queue;
    bool threaded;

    // Only used if threaded == false
    boost::mutex st_mutex;
    /// @todo Could get rid of st_mutex by updating [thread_index|in_thread] atomically
    uint32_t thread_index;
    uint32_t in_thread;
  };
  typedef boost::shared_ptr<QueueInfo> QueueInfoPtr;

  typedef boost::unordered_map<CallbackQueue*, QueueInfoPtr> M_Queue;
  M_Queue queues_;
  boost::mutex queues_mutex_;

  /// @todo SRMW lockfree queue. waiting_mutex_ has the potential for a lot of contention
  typedef std::vector<CallbackQueuePtr> V_Queue;
  V_Queue waiting_;
  boost::mutex waiting_mutex_;
  boost::condition_variable waiting_cond_;
  boost::thread_group tg_;

  struct ThreadInfo
  {
    ThreadInfo()
    : calling(0)
    {}

    /// @todo SRSW lockfree queue
    boost::mutex queue_mutex;
    boost::condition_variable queue_cond;
    std::vector<std::pair<CallbackQueuePtr, QueueInfoPtr> > queue;
    boost::detail::atomic_count calling;

#ifdef NODELET_QUEUE_DEBUG
    struct Record
    {
      Record(double stamp, uint32_t tasks, bool threaded)
        : stamp(stamp), tasks(tasks), threaded(threaded)
      {}
      
      double stamp;
      uint32_t tasks;
      bool threaded;
    };

    std::vector<Record> history;
#endif

    // Pad sizeof(ThreadInfo) to be a multiple of 64 (cache line size) to avoid false sharing.
    // This still doesn't guarantee ThreadInfo is actually allocated on a cache line boundary though.
    static const int ACTUAL_SIZE =
      sizeof(boost::mutex) +
      sizeof(boost::condition_variable) +
      sizeof(std::vector<std::pair<CallbackQueuePtr, QueueInfoPtr> >) +
      sizeof(boost::detail::atomic_count);
    uint8_t pad[((ACTUAL_SIZE + 63) & ~63) - ACTUAL_SIZE];
  };
  /// @todo Use cache-aligned allocator for thread_info_
  typedef boost::scoped_array<ThreadInfo> V_ThreadInfo;
  V_ThreadInfo thread_info_;

  bool running_;
  uint32_t num_worker_threads_;
};

} // namespace detail
} // namespace nodelet

#endif // NODELET_CALLBACK_QUEUE_MANAGER_H
