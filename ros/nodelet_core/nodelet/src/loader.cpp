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

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <nodelet/detail/callback_queue.h>
#include <nodelet/detail/callback_queue_manager.h>
#include <pluginlib/class_loader.h>
#include <bondcpp/bond.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletList.h>
#include <nodelet/NodeletUnload.h>

#include <boost/ptr_container/ptr_map.hpp>
#include <boost/utility.hpp>

/*
Between Loader, Nodelet, CallbackQueue and CallbackQueueManager, who owns what?

Loader contains the CallbackQueueManager. Loader and CallbackQueueManager share
ownership of the CallbackQueues. Loader is primary owner of the Nodelets, but
each CallbackQueue has weak ownership of its associated Nodelet.

Loader ensures that the CallbackQueues associated with a Nodelet outlive that
Nodelet. CallbackQueueManager ensures that CallbackQueues continue to survive as
long as they have pending callbacks. 

CallbackQueue holds a weak_ptr to its associated Nodelet, which it attempts to
lock before invoking any callback. So any lingering callbacks processed after a
Nodelet is destroyed are safely discarded, and then the CallbackQueue expires.

When Loader unloads a Nodelet, it calls CallbackQueueManager::removeQueue() for
the associated CallbackQueues, which prevents them from adding any more callbacks
to CallbackQueueManager. Otherwise a Nodelet that is continuously executing
callbacks might persist in a "zombie" state after being unloaded.
 */

namespace nodelet
{

typedef boost::shared_ptr<Nodelet> NodeletPtr;

/// @todo Consider moving this to nodelet executable, it's implemented entirely on top of Loader
class LoaderROS
{
public:
  LoaderROS(Loader* parent, const ros::NodeHandle& nh)
  : parent_(parent)
  , nh_(nh)
  , bond_spinner_(1, &bond_callback_queue_)
  {
    load_server_ = nh_.advertiseService("load_nodelet", &LoaderROS::serviceLoad, this);
    unload_server_ = nh_.advertiseService("unload_nodelet", &LoaderROS::serviceUnload, this);
    list_server_ = nh_.advertiseService("list", &LoaderROS::serviceList, this);

    bond_spinner_.start();
  }

private:
  bool serviceLoad(nodelet::NodeletLoad::Request &req,
                   nodelet::NodeletLoad::Response &res)
  {
    boost::mutex::scoped_lock lock(lock_);

    // build map
    M_string remappings;
    if (req.remap_source_args.size() != req.remap_target_args.size())
    {
      ROS_ERROR("Bad remapppings provided, target and source of different length");
    }
    else
    {
      for (size_t i = 0; i < req.remap_source_args.size(); ++i)
      {
        remappings[ros::names::resolve(req.remap_source_args[i])] = ros::names::resolve(req.remap_target_args[i]);
        ROS_DEBUG("%s:%s\n", ros::names::resolve(req.remap_source_args[i]).c_str(), remappings[ros::names::resolve(req.remap_source_args[i])].c_str());
      }
    }

    res.success = parent_->load(req.name, req.type, remappings, req.my_argv);

    // If requested, create bond to sister process
    if (res.success && !req.bond_id.empty())
    {
      bond::Bond* bond = new bond::Bond(nh_.getNamespace() + "/bond", req.bond_id);
      bond_map_.insert(req.name, bond);
      bond->setCallbackQueue(&bond_callback_queue_);
      bond->setBrokenCallback(boost::bind(&LoaderROS::unload, this, req.name));
      bond->start();
    }
    return res.success;
  }

  bool serviceUnload(nodelet::NodeletUnload::Request &req,
                     nodelet::NodeletUnload::Response &res)
  {
    res.success = unload(req.name);
    return res.success;
  }

  bool unload(const std::string& name)
  {
    boost::mutex::scoped_lock lock(lock_);

    bool success = parent_->unload(name);
    if (!success)
    {
      ROS_ERROR("Failed to find nodelet with name '%s' to unload.", name.c_str());
      return success;
    }

    // break the bond, if there is one
    M_stringToBond::iterator it = bond_map_.find(name);
    if (it != bond_map_.end()) {
        // disable callback for broken bond, as we are breaking it intentially now
        it->second->setBrokenCallback(boost::function<void(void)>());
        // erase (and break) bond
        bond_map_.erase(name);
    }

    return success;
  }

  bool serviceList(nodelet::NodeletList::Request &,
                   nodelet::NodeletList::Response &res)
  {
    res.nodelets = parent_->listLoadedNodelets();
    return true;
  }

  Loader* parent_;
  ros::NodeHandle nh_;
  ros::ServiceServer load_server_;
  ros::ServiceServer unload_server_;
  ros::ServiceServer list_server_;

  boost::mutex lock_;

  ros::CallbackQueue bond_callback_queue_;
  ros::AsyncSpinner bond_spinner_;
  typedef boost::ptr_map<std::string, bond::Bond> M_stringToBond;
  M_stringToBond bond_map_;
};

// Owns a Nodelet and its callback queues
struct ManagedNodelet : boost::noncopyable
{
  detail::CallbackQueuePtr st_queue;
  detail::CallbackQueuePtr mt_queue;
  NodeletPtr nodelet; // destroyed before the queues
  detail::CallbackQueueManager* callback_manager;

  /// @todo Maybe addQueue/removeQueue should be done by CallbackQueue
  ManagedNodelet(const NodeletPtr& nodelet, detail::CallbackQueueManager* cqm)
    : st_queue(new detail::CallbackQueue(cqm, nodelet))
    , mt_queue(new detail::CallbackQueue(cqm, nodelet))
    , nodelet(nodelet)
    , callback_manager(cqm)
  {
    // NOTE: Can't do this in CallbackQueue constructor because the shared_ptr to
    // it doesn't exist then.
    callback_manager->addQueue(st_queue, false);
    callback_manager->addQueue(mt_queue, true);
  }

  ~ManagedNodelet()
  {
    callback_manager->removeQueue(st_queue);
    callback_manager->removeQueue(mt_queue);
  }
};

struct Loader::Impl
{
  boost::shared_ptr<LoaderROS> services_;

  boost::function<boost::shared_ptr<Nodelet> (const std::string& lookup_name)> create_instance_;
  boost::function<void ()> refresh_classes_;
  boost::shared_ptr<detail::CallbackQueueManager> callback_manager_; // Must outlive nodelets_

  typedef boost::ptr_map<std::string, ManagedNodelet> M_stringToNodelet;
  M_stringToNodelet nodelets_; ///<! A map of name to currently constructed nodelets

  Impl()
  {
    // Under normal circumstances, we use pluginlib to load any registered nodelet
    typedef pluginlib::ClassLoader<Nodelet> Loader;
    boost::shared_ptr<Loader> loader(new Loader("nodelet", "nodelet::Nodelet"));
    // create_instance_ is self-contained; it owns a copy of the loader shared_ptr
    create_instance_ = boost::bind(&Loader::createInstance, loader, _1);
    refresh_classes_ = boost::bind(&Loader::refreshDeclaredClasses, loader);
  }

  Impl(const boost::function<boost::shared_ptr<Nodelet> (const std::string& lookup_name)>& create_instance)
    : create_instance_(create_instance)
  {
  }

  void advertiseRosApi(Loader* parent, const ros::NodeHandle& server_nh)
  {
    int num_threads_param;
    server_nh.param("num_worker_threads", num_threads_param, 0);
    callback_manager_.reset(new detail::CallbackQueueManager(num_threads_param));
    ROS_INFO("Initializing nodelet with %d worker threads.", (int)callback_manager_->getNumWorkerThreads());
  
    services_.reset(new LoaderROS(parent, server_nh));
  }
};

/// @todo Instance of ROS API-related constructors, just take #threads for the manager
Loader::Loader(bool provide_ros_api)
  : impl_(new Impl)
{
  if (provide_ros_api)
    impl_->advertiseRosApi(this, ros::NodeHandle("~"));
  else
    impl_->callback_manager_.reset(new detail::CallbackQueueManager);
}

Loader::Loader(const ros::NodeHandle& server_nh)
  : impl_(new Impl)
{
  impl_->advertiseRosApi(this, server_nh);
}

Loader::Loader(const boost::function<boost::shared_ptr<Nodelet> (const std::string& lookup_name)>& create_instance)
  : impl_(new Impl(create_instance))
{
  impl_->callback_manager_.reset(new detail::CallbackQueueManager);
}

Loader::~Loader()
{
}

bool Loader::load(const std::string &name, const std::string& type, const ros::M_string& remappings,
                  const std::vector<std::string> & my_argv)
{
  boost::mutex::scoped_lock lock(lock_);
  if (impl_->nodelets_.count(name) > 0)
  {
    ROS_ERROR("Cannot load nodelet %s for one exists with that name already", name.c_str());
    return false;
  }

  NodeletPtr p;
  try
  {
    p = impl_->create_instance_(type);
  }
  catch (std::runtime_error& e)
  {
    // If we cannot refresh the nodelet cache, fail immediately
    if(!impl_->refresh_classes_)
    {
      ROS_ERROR("Failed to load nodelet [%s] of type [%s]: %s", name.c_str(), type.c_str(), e.what());
      return false;
    }

    // otherwise, refresh the cache and try again.
    try
    {
      impl_->refresh_classes_();
      p = impl_->create_instance_(type);
    }
    catch (std::runtime_error& e2)
    {
      // dlopen() can return inconsistent results currently (see
      // https://sourceware.org/bugzilla/show_bug.cgi?id=17833), so make sure
      // that we display the messages of both exceptions to the user.
      ROS_ERROR("Failed to load nodelet [%s] of type [%s] even after refreshing the cache: %s", name.c_str(), type.c_str(), e2.what());
      ROS_ERROR("The error before refreshing the cache was: %s", e.what());
      return false;
    }
  }

  if (!p)
  {
    return false;
  }
  ROS_DEBUG("Done loading nodelet %s", name.c_str());

  ManagedNodelet* mn = new ManagedNodelet(p, impl_->callback_manager_.get());
  impl_->nodelets_.insert(const_cast<std::string&>(name), mn); // mn now owned by boost::ptr_map
  try {
    p->init(name, remappings, my_argv, mn->st_queue.get(), mn->mt_queue.get());
    /// @todo Can we delay processing the queues until Nodelet::onInit() returns?

    ROS_DEBUG("Done initing nodelet %s", name.c_str());
  } catch(...) {
    Impl::M_stringToNodelet::iterator it = impl_->nodelets_.find(name);
    if (it != impl_->nodelets_.end())
    {
      impl_->nodelets_.erase(it);
      ROS_DEBUG ("Failed to initialize nodelet %s", name.c_str ());
      return (false);
    }
  }
  return true;
}

bool Loader::unload (const std::string & name)
{
  boost::mutex::scoped_lock lock (lock_);
  Impl::M_stringToNodelet::iterator it = impl_->nodelets_.find(name);
  if (it != impl_->nodelets_.end())
  {
    impl_->nodelets_.erase(it);
    ROS_DEBUG ("Done unloading nodelet %s", name.c_str ());
    return (true);
  }

  return (false);
}

bool Loader::clear ()
{
  boost::mutex::scoped_lock lock(lock_);
  impl_->nodelets_.clear();
  return true;
};

std::vector<std::string> Loader::listLoadedNodelets()
{
  boost::mutex::scoped_lock lock (lock_);
  std::vector<std::string> output;
  Impl::M_stringToNodelet::iterator it = impl_->nodelets_.begin();
  for (; it != impl_->nodelets_.end(); ++it)
  {
    output.push_back(it->first);
  }
  return output;
}

} // namespace nodelet

