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

#ifndef NODELET_NODELET_H
#define NODELET_NODELET_H

#include "exception.h"

#include <string>
#include <vector>
#include <map>

#include <ros/console.h>
#include <boost/shared_ptr.hpp>

namespace ros
{
class NodeHandle;
class CallbackQueueInterface;
}

#define NODELET_DEBUG(...) ROS_DEBUG_NAMED(getName(), __VA_ARGS__)
#define NODELET_DEBUG_STREAM(...) ROS_DEBUG_STREAM_NAMED(getName(), __VA_ARGS__)
#define NODELET_DEBUG_ONCE(...) ROS_DEBUG_ONCE_NAMED(getName(), __VA_ARGS__)
#define NODELET_DEBUG_STREAM_ONCE(...) ROS_DEBUG_STREAM_ONCE_NAMED(getName(), __VA_ARGS__)
#define NODELET_DEBUG_COND(cond, ...) ROS_DEBUG_COND_NAMED(cond, getName(), __VA_ARGS__)
#define NODELET_DEBUG_STREAM_COND(cond, ...) ROS_DEBUG_STREAM_COND_NAMED(cond, getName(), __VA_ARGS__)
#define NODELET_DEBUG_THROTTLE(rate, ...) ROS_DEBUG_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_DEBUG_STREAM_THROTTLE(rate, ...) ROS_DEBUG_STREAM_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_DEBUG_FILTER(filter, ...) ROS_DEBUG_FILTER_NAMED(filter, getName(), __VA_ARGS__)
#define NODELET_DEBUG_STREAM_FILTER(filter, ...) ROS_DEBUG_STREAM_FILTER_NAMED(filter, getName(), __VA_ARGS__)

#define NODELET_INFO(...) ROS_INFO_NAMED(getName(), __VA_ARGS__)
#define NODELET_INFO_STREAM(...) ROS_INFO_STREAM_NAMED(getName(), __VA_ARGS__)
#define NODELET_INFO_ONCE(...) ROS_INFO_ONCE_NAMED(getName(), __VA_ARGS__)
#define NODELET_INFO_STREAM_ONCE(...) ROS_INFO_STREAM_ONCE_NAMED(getName(), __VA_ARGS__)
#define NODELET_INFO_COND(cond, ...) ROS_INFO_COND_NAMED(cond, getName(), __VA_ARGS__)
#define NODELET_INFO_STREAM_COND(cond, ...) ROS_INFO_STREAM_COND_NAMED(cond, getName(), __VA_ARGS__)
#define NODELET_INFO_THROTTLE(rate, ...) ROS_INFO_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_INFO_STREAM_THROTTLE(rate, ...) ROS_INFO_STREAM_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_INFO_FILTER(filter, ...) ROS_INFO_FILTER_NAMED(filter, getName(), __VA_ARGS__)
#define NODELET_INFO_STREAM_FILTER(filter, ...) ROS_INFO_STREAM_FILTER_NAMED(filter, getName(), __VA_ARGS__)

#define NODELET_WARN(...) ROS_WARN_NAMED(getName(), __VA_ARGS__)
#define NODELET_WARN_STREAM(...) ROS_WARN_STREAM_NAMED(getName(), __VA_ARGS__)
#define NODELET_WARN_ONCE(...) ROS_WARN_ONCE_NAMED(getName(), __VA_ARGS__)
#define NODELET_WARN_STREAM_ONCE(...) ROS_WARN_STREAM_ONCE_NAMED(getName(), __VA_ARGS__)
#define NODELET_WARN_COND(cond, ...) ROS_WARN_COND_NAMED(cond, getName(), __VA_ARGS__)
#define NODELET_WARN_STREAM_COND(cond, ...) ROS_WARN_STREAM_COND_NAMED(cond, getName(), __VA_ARGS__)
#define NODELET_WARN_THROTTLE(rate, ...) ROS_WARN_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_WARN_STREAM_THROTTLE(rate, ...) ROS_WARN_STREAM_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_WARN_FILTER(filter, ...) ROS_WARN_FILTER_NAMED(filter, getName(), __VA_ARGS__)
#define NODELET_WARN_STREAM_FILTER(filter, ...) ROS_WARN_STREAM_FILTER_NAMED(filter, getName(), __VA_ARGS__)

#define NODELET_ERROR(...) ROS_ERROR_NAMED(getName(), __VA_ARGS__)
#define NODELET_ERROR_STREAM(...) ROS_ERROR_STREAM_NAMED(getName(), __VA_ARGS__)
#define NODELET_ERROR_ONCE(...) ROS_ERROR_ONCE_NAMED(getName(), __VA_ARGS__)
#define NODELET_ERROR_STREAM_ONCE(...) ROS_ERROR_STREAM_ONCE_NAMED(getName(), __VA_ARGS__)
#define NODELET_ERROR_COND(cond, ...) ROS_ERROR_COND_NAMED(cond, getName(), __VA_ARGS__)
#define NODELET_ERROR_STREAM_COND(cond, ...) ROS_ERROR_STREAM_COND_NAMED(cond, getName(), __VA_ARGS__)
#define NODELET_ERROR_THROTTLE(rate, ...) ROS_ERROR_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_ERROR_STREAM_THROTTLE(rate, ...) ROS_ERROR_STREAM_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_ERROR_FILTER(filter, ...) ROS_ERROR_FILTER_NAMED(filter, getName(), __VA_ARGS__)
#define NODELET_ERROR_STREAM_FILTER(filter, ...) ROS_ERROR_STREAM_FILTER_NAMED(filter, getName(), __VA_ARGS__)

#define NODELET_FATAL(...) ROS_FATAL_NAMED(getName(), __VA_ARGS__)
#define NODELET_FATAL_STREAM(...) ROS_FATAL_STREAM_NAMED(getName(), __VA_ARGS__)
#define NODELET_FATAL_ONCE(...) ROS_FATAL_ONCE_NAMED(getName(), __VA_ARGS__)
#define NODELET_FATAL_STREAM_ONCE(...) ROS_FATAL_STREAM_ONCE_NAMED(getName(), __VA_ARGS__)
#define NODELET_FATAL_COND(cond, ...) ROS_FATAL_COND_NAMED(cond, getName(), __VA_ARGS__)
#define NODELET_FATAL_STREAM_COND(cond, ...) ROS_FATAL_STREAM_COND_NAMED(cond, getName(), __VA_ARGS__)
#define NODELET_FATAL_THROTTLE(rate, ...) ROS_FATAL_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_FATAL_STREAM_THROTTLE(rate, ...) ROS_FATAL_STREAM_THROTTLE_NAMED(rate, getName(), __VA_ARGS__)
#define NODELET_FATAL_FILTER(filter, ...) ROS_FATAL_FILTER_NAMED(filter, getName(), __VA_ARGS__)
#define NODELET_FATAL_STREAM_FILTER(filter, ...) ROS_FATAL_STREAM_FILTER_NAMED(filter, getName(), __VA_ARGS__)

// named versions of the macros
#define NODELET_DEBUG_NAMED(suffix, ...) ROS_DEBUG_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_DEBUG_STREAM_NAMED(suffix, ...) ROS_DEBUG_STREAM_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_DEBUG_ONCE_NAMED(suffix, ...) ROS_DEBUG_ONCE_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_DEBUG_STREAM_ONCE_NAMED(suffix, ...) ROS_DEBUG_STREAM_ONCE_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_DEBUG_COND_NAMED(cond, suffix, ...) ROS_DEBUG_COND_NAMED(cond, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_DEBUG_STREAM_COND_NAMED(cond, suffix, ...) ROS_DEBUG_STREAM_COND_NAMED(cond, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_DEBUG_THROTTLE_NAMED(rate, suffix, ...) ROS_DEBUG_THROTTLE_NAMED(rate, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_DEBUG_STREAM_THROTTLE_NAMED(rate, suffix, ...) ROS_DEBUG_STREAM_THROTTLE_NAMED(rate, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_DEBUG_FILTER_NAMED(filter, suffix, ...) ROS_DEBUG_FILTER_NAMED(filter, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_DEBUG_STREAM_FILTER_NAMED(filter, suffix, ...) ROS_DEBUG_STREAM_FILTER_NAMED(filter, getSuffixedName(suffix), __VA_ARGS__)

#define NODELET_INFO_NAMED(suffix, ...) ROS_INFO_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_INFO_STREAM_NAMED(suffix, ...) ROS_INFO_STREAM_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_INFO_ONCE_NAMED(suffix, ...) ROS_INFO_ONCE_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_INFO_STREAM_ONCE_NAMED(suffix, ...) ROS_INFO_STREAM_ONCE_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_INFO_COND_NAMED(cond, suffix, ...) ROS_INFO_COND_NAMED(cond, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_INFO_STREAM_COND_NAMED(cond, suffix, ...) ROS_INFO_STREAM_COND_NAMED(cond, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_INFO_THROTTLE_NAMED(rate, suffix, ...) ROS_INFO_THROTTLE_NAMED(rate, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_INFO_STREAM_THROTTLE_NAMED(rate, suffix, ...) ROS_INFO_STREAM_THROTTLE_NAMED(rate, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_INFO_FILTER_NAMED(filter, suffix, ...) ROS_INFO_FILTER_NAMED(filter, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_INFO_STREAM_FILTER_NAMED(filter, suffix, ...) ROS_INFO_STREAM_FILTER_NAMED(filter, getSuffixedName(suffix), __VA_ARGS__)

#define NODELET_WARN_NAMED(suffix, ...) ROS_WARN_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_WARN_STREAM_NAMED(suffix, ...) ROS_WARN_STREAM_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_WARN_ONCE_NAMED(suffix, ...) ROS_WARN_ONCE_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_WARN_STREAM_ONCE_NAMED(suffix, ...) ROS_WARN_STREAM_ONCE_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_WARN_COND_NAMED(cond, suffix, ...) ROS_WARN_COND_NAMED(cond, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_WARN_STREAM_COND_NAMED(cond, suffix, ...) ROS_WARN_STREAM_COND_NAMED(cond, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_WARN_THROTTLE_NAMED(rate, suffix, ...) ROS_WARN_THROTTLE_NAMED(rate, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_WARN_STREAM_THROTTLE_NAMED(rate, suffix, ...) ROS_WARN_STREAM_THROTTLE_NAMED(rate, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_WARN_FILTER_NAMED(filter, suffix, ...) ROS_WARN_FILTER_NAMED(filter, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_WARN_STREAM_FILTER_NAMED(filter, suffix, ...) ROS_WARN_STREAM_FILTER_NAMED(filter, getSuffixedName(suffix), __VA_ARGS__)

#define NODELET_ERROR_NAMED(suffix, ...) ROS_ERROR_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_ERROR_STREAM_NAMED(suffix, ...) ROS_ERROR_STREAM_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_ERROR_ONCE_NAMED(suffix, ...) ROS_ERROR_ONCE_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_ERROR_STREAM_ONCE_NAMED(suffix, ...) ROS_ERROR_STREAM_ONCE_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_ERROR_COND_NAMED(cond, suffix, ...) ROS_ERROR_COND_NAMED(cond, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_ERROR_STREAM_COND_NAMED(cond, suffix, ...) ROS_ERROR_STREAM_COND_NAMED(cond, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_ERROR_THROTTLE_NAMED(rate, suffix, ...) ROS_ERROR_THROTTLE_NAMED(rate, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_ERROR_STREAM_THROTTLE_NAMED(rate, suffix, ...) ROS_ERROR_STREAM_THROTTLE_NAMED(rate, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_ERROR_FILTER_NAMED(filter, suffix, ...) ROS_ERROR_FILTER_NAMED(filter, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_ERROR_STREAM_FILTER_NAMED(filter, suffix, ...) ROS_ERROR_STREAM_FILTER_NAMED(filter, getSuffixedName(suffix), __VA_ARGS__)

#define NODELET_FATAL_NAMED(suffix, ...) ROS_FATAL_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_FATAL_STREAM_NAMED(suffix, ...) ROS_FATAL_STREAM_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_FATAL_ONCE_NAMED(suffix, ...) ROS_FATAL_ONCE_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_FATAL_STREAM_ONCE_NAMED(suffix, ...) ROS_FATAL_STREAM_ONCE_NAMED(getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_FATAL_COND_NAMED(cond, suffix, ...) ROS_FATAL_COND_NAMED(cond, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_FATAL_STREAM_COND_NAMED(cond, suffix, ...) ROS_FATAL_STREAM_COND_NAMED(cond, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_FATAL_THROTTLE_NAMED(rate, suffix, ...) ROS_FATAL_THROTTLE_NAMED(rate, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_FATAL_STREAM_THROTTLE_NAMED(rate, suffix, ...) ROS_FATAL_STREAM_THROTTLE_NAMED(rate, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_FATAL_FILTER_NAMED(filter, suffix, ...) ROS_FATAL_FILTER_NAMED(filter, getSuffixedName(suffix), __VA_ARGS__)
#define NODELET_FATAL_STREAM_FILTER_NAMED(filter, suffix, ...) ROS_FATAL_STREAM_FILTER_NAMED(filter, getSuffixedName(suffix), __VA_ARGS__)

namespace nodelet
{
typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
typedef std::map<std::string, std::string> M_string;
typedef std::vector<std::string> V_string;

class UninitializedException : public Exception
{
public:
  UninitializedException(const std::string& method_name)
  : Exception("Calling [" + method_name + "] before the Nodelet is initialized is not allowed.")
  {}
};

class MultipleInitializationException : public Exception
{
public:
  MultipleInitializationException()
  : Exception("Initialized multiple times")
  {}
};

class Nodelet
{
  // Protected data fields for use by the subclass.
protected:
  inline const std::string& getName() const { return nodelet_name_; }
  inline std::string getSuffixedName(const std::string& suffix) const
  {
    return nodelet_name_ + "." + suffix;
  }
  inline const V_string& getMyArgv() const { return my_argv_; }

  ros::NodeHandle& getNodeHandle() const;
  ros::NodeHandle& getPrivateNodeHandle() const;
  ros::NodeHandle& getMTNodeHandle() const;
  ros::NodeHandle& getMTPrivateNodeHandle() const;

  ros::CallbackQueueInterface& getSTCallbackQueue() const;
  ros::CallbackQueueInterface& getMTCallbackQueue() const;


  // Internal storage;
private:
  bool inited_;

  std::string nodelet_name_;

  NodeHandlePtr nh_;
  NodeHandlePtr private_nh_;
  NodeHandlePtr mt_nh_;
  NodeHandlePtr mt_private_nh_;
  V_string my_argv_;

  // Method to be overridden by subclass when starting up.
  virtual void onInit() = 0;

  // Public API used for launching
public:
  /**\brief Empty constructor required for dynamic loading */
  Nodelet();

  /**\brief Init function called at startup
   * \param name The name of the nodelet
   * \param remapping_args The remapping args in a map for the nodelet
   * \param my_argv The commandline arguments for this nodelet stripped of special arguments such as ROS arguments
   */
  void init(const std::string& name, const M_string& remapping_args, const V_string& my_argv,
            ros::CallbackQueueInterface* st_queue = NULL,
            ros::CallbackQueueInterface* mt_queue = NULL);

  virtual ~Nodelet();
};

}
#endif //NODELET_NODELET_H
