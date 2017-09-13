/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009-2010, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/


/**

 Author: Blaise Gassend

 Handles synchronizing node state with the configuration server, and
 handling of services to get and set configuration.

*/

#ifndef __SERVER_H__
#define __SERVER_H__

#include <boost/function.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>

/**
 * @todo Add diagnostics.
 */

namespace dynamic_reconfigure
{
/**
 * Keeps track of the reconfigure callback function.
 */
template <class ConfigType>
class Server
{
public:
  Server(const ros::NodeHandle &nh = ros::NodeHandle("~")) :
    node_handle_(nh),
    mutex_(own_mutex_),
    own_mutex_warn_(true)
  {
    init();
  }

  Server(boost::recursive_mutex &mutex, const ros::NodeHandle &nh = ros::NodeHandle("~")) :
    node_handle_(nh),
    mutex_(mutex),
    own_mutex_warn_(false)
  {
    init();
  }

  typedef boost::function<void(ConfigType &, uint32_t level)> CallbackType;

  void setCallback(const CallbackType &callback)
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    callback_ = callback;
    callCallback(config_, ~0); // At startup we need to load the configuration with all level bits set. (Everything has changed.)
    updateConfigInternal(config_);
  }

  void clearCallback()
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    callback_.clear();
  }

  void updateConfig(const ConfigType &config)
  {
    if (own_mutex_warn_)
    {
      ROS_WARN("updateConfig() called on a dynamic_reconfigure::Server that provides its own mutex. This can lead to deadlocks if updateConfig() is called during an update. Providing a mutex to the constructor is highly recommended in this case. Please forward this message to the node author.");
      own_mutex_warn_ = false;
    }
    updateConfigInternal(config);
  }


  void getConfigMax(ConfigType &config)
  {
    config = max_;
  }

  void getConfigMin(ConfigType &config)
  {
    config = min_;
  }

  void getConfigDefault(ConfigType &config)
  {
    config = default_;
  }

  void setConfigMax(const ConfigType &config)
  {
    max_ = config;
    PublishDescription();
  }

  void setConfigMin(const ConfigType &config)
  {
    min_ = config;
    PublishDescription();
  }

  void setConfigDefault(const ConfigType &config)
  {
    default_ = config;
    PublishDescription();
  }


private:
  ros::NodeHandle node_handle_;
  ros::ServiceServer set_service_;
  ros::Publisher update_pub_;
  ros::Publisher descr_pub_;
  CallbackType callback_;
  ConfigType config_;
  ConfigType min_;
  ConfigType max_;
  ConfigType default_;
  boost::recursive_mutex &mutex_;
  boost::recursive_mutex own_mutex_; // Used only if an external one isn't specified.
  bool own_mutex_warn_;



  void PublishDescription()
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    //Copy over min_ max_ default_
    dynamic_reconfigure::ConfigDescription description_message = ConfigType::__getDescriptionMessage__();

    max_.__toMessage__(description_message.max, ConfigType::__getParamDescriptions__(),ConfigType::__getGroupDescriptions__());
    min_.__toMessage__(description_message.min,ConfigType::__getParamDescriptions__(),ConfigType::__getGroupDescriptions__());
    default_.__toMessage__(description_message.dflt,ConfigType::__getParamDescriptions__(),ConfigType::__getGroupDescriptions__());

    //Publish description
    descr_pub_.publish(description_message);
  }

  void init()
  {
    //Grab copys of the data from the config files.  These are declared in the generated config file.
    min_ = ConfigType::__getMin__();
    max_ = ConfigType::__getMax__();
    default_ = ConfigType::__getDefault__();

    boost::recursive_mutex::scoped_lock lock(mutex_);
    set_service_ = node_handle_.advertiseService("set_parameters",
        &Server<ConfigType>::setConfigCallback, this);

    descr_pub_ = node_handle_.advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);
    descr_pub_.publish(ConfigType::__getDescriptionMessage__());

    update_pub_ = node_handle_.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);
    ConfigType init_config = ConfigType::__getDefault__();
    init_config.__fromServer__(node_handle_);
    init_config.__clamp__();
    updateConfigInternal(init_config);
  }

  void callCallback(ConfigType &config, int level)
  {
    if (callback_) // At startup we need to load the configuration with all level bits set. (Everything has changed.)
      try {
        callback_(config, level);
      }
      catch (std::exception &e)
      {
        ROS_WARN("Reconfigure callback failed with exception %s: ", e.what());
      }
      catch (...)
      {
        ROS_WARN("Reconfigure callback failed with unprintable exception.");
      }
    else
      ROS_DEBUG("setCallback did not call callback because it was zero."); /// @todo kill this line.
  }

  bool setConfigCallback(dynamic_reconfigure::Reconfigure::Request &req,
          dynamic_reconfigure::Reconfigure::Response &rsp)
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);

    ConfigType new_config = config_;
    new_config.__fromMessage__(req.config);
    new_config.__clamp__();
    uint32_t level = config_.__level__(new_config);

    callCallback(new_config, level);

    updateConfigInternal(new_config);
    new_config.__toMessage__(rsp.config);
    return true;
  }

  void updateConfigInternal(const ConfigType &config)
  {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    config_ = config;
    config_.__toServer__(node_handle_);
    dynamic_reconfigure::Config msg;
    config_.__toMessage__(msg);
    update_pub_.publish(msg);
  }
};

}
#endif
