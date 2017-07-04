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

/**
@mainpage

\author Tully Foote
**/

#ifndef NODELET_LOADER_H
#define NODELET_LOADER_H

#include <map>
#include <vector>
#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

namespace ros
{
class NodeHandle;
}

namespace nodelet
{
class Nodelet;
typedef std::map<std::string, std::string> M_string;
typedef std::vector<std::string> V_string;

/** \brief A class which will construct and sequentially call Nodelets according to xml
 * This is the primary way in which users are expected to interact with Nodelets
 */
class Loader
{
public:
  /** \brief Construct the nodelet loader with optional ros API at default location of NodeHandle("~")*/
  Loader(bool provide_ros_api = true);
  /** \brief Construct the nodelet loader with optional ros API in namespace of passed NodeHandle */
  Loader(const ros::NodeHandle& server_nh);
  /**
   * \brief Construct the nodelet loader without ros API, using non-standard factory function to
   * create nodelet instances
   */
  Loader(const boost::function<boost::shared_ptr<Nodelet> (const std::string& lookup_name)>& create_instance);
  
  ~Loader();

  /** \brief Load a nodelet */
  bool load(const std::string& name, const std::string& type, const M_string& remappings,
            const V_string& my_argv);

  /** \brief Unload a nodelet */
  bool unload(const std::string& name);

  /** \brief Clear all nodelets from this loader */
  bool clear();

  /**\brief List the names of all loaded nodelets */
  std::vector<std::string> listLoadedNodelets();
  
private:
  boost::mutex lock_; ///<! Public methods must lock this to preserve internal integrity.
  struct Impl;
  boost::scoped_ptr<Impl> impl_;
};

} // namespace nodelet

#endif //#ifndef NODELET_LOADER_H
