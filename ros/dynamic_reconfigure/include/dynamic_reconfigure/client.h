/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015-2016, Myrmex, Inc.
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
*   * Neither the name of the Myrmex, Inc nor the names of its
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

 Author: Aris Synodinos

 Handles sychronizing node state with the configuration server and setting/getting
 configuration.

 */

#ifndef __CLIENT_H__
#define __CLIENT_H__

#include <boost/chrono.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>

namespace dynamic_reconfigure {

template <class ConfigType>
class Client {
 public:
  /**
   * @brief Client Constructs a statefull dynamic_reconfigure client
   * @param name The full path of the dynamic_reconfigure::Server
   * @param config_callback A callback that should be called when the server
   * informs the clients of a successful reconfiguration
   * @param description_callback A callback that should be called when the
   * server infrorms the clients of the description of the reconfiguration
   * parameters and groups
   */
  Client(
      const std::string& name,
      const boost::function<void(const ConfigType&)> config_callback = 0,
      const boost::function<void(const dynamic_reconfigure::ConfigDescription&)>
          description_callback = 0)
      : name_(name),
        nh_(name),
        received_configuration_(false),
        received_description_(false),
        config_callback_(config_callback),
        description_callback_(description_callback) {
    set_service_ =
        nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters");

    config_sub_ =
        nh_.subscribe("parameter_updates", 1,
                      &Client<ConfigType>::configurationCallback, this);

    descr_sub_ = nh_.subscribe("parameter_descriptions", 1,
                               &Client<ConfigType>::descriptionCallback, this);
  }
  /**
   * @brief Client Constructs a statefull dynamic_reconfigure client
   * @param name The full path of the dynamic_reconfigure::Server
   * @param nh The nodehandle to the full path of the Server (for nodelets)
   * @param config_callback A callback that should be called when the server
   * informs the clients of a successful reconfiguration
   * @param description_callback A callback that should be called when the
   * server infrorms the clients of the description of the reconfiguration
   * parameters and groups
   */
  Client(
      const std::string& name, const ros::NodeHandle& nh,
      const boost::function<void(const ConfigType&)> config_callback = 0,
      const boost::function<void(const dynamic_reconfigure::ConfigDescription&)>
          description_callback = 0)
      : name_(name),
        nh_(nh),
        received_configuration_(false),
        received_description_(false),
        config_callback_(config_callback),
        description_callback_(description_callback) {
    set_service_ =
        nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters");

    config_sub_ =
        nh_.subscribe("parameter_updates", 1,
                      &Client<ConfigType>::configurationCallback, this);

    descr_sub_ = nh_.subscribe("parameter_descriptions", 1,
                               &Client<ConfigType>::descriptionCallback, this);
  }
  /**
   * @brief setConfigurationCallback Sets the user defined configuration
   * callback function
   * @param config_callback A function pointer
   */
  void setConfigurationCallback(
      const boost::function<void(const ConfigType&)>& config_callback) {
    config_callback_ = config_callback;
  }
  /**
   * @brief setDescriptionCallback Sets the user defined description callback
   * function
   * @param description_callback A function pointer
   */
  void setDescriptionCallback(const boost::function<void(
      const dynamic_reconfigure::ConfigDescription&)>& description_callback) {
    description_callback_ = description_callback;
  }
  /**
   * @brief setConfiguration Attempts to set the configuration to the server
   * @param configuration The requested configuration
   * @return True if the server accepted the request (not the reconfiguration)
   */
  bool setConfiguration(const ConfigType& configuration) {
    ConfigType temp = configuration;
    return setConfiguration(temp);
  }
  /**
   * @brief setConfiguration Attempts to set the configuration to the server
   * @param configuration The requested configuration, gets overwritten with the
   * reply from the reconfigure server
   * @return True if the server accepted the request (not the reconfiguration)
   */
  bool setConfiguration(ConfigType& configuration) {
    dynamic_reconfigure::Reconfigure srv;
    received_configuration_ = false;
    configuration.__toMessage__(srv.request.config);
    if (set_service_.call(srv)) {
      configuration.__fromMessage__(srv.response.config);
      return true;
    } else {
      ROS_WARN("Could not set configuration");
      return false;
    }
  }
  /**
   * @brief getCurrentConfiguration Gets the latest configuration from the
   * dynamic_reconfigure::Server
   * @param configuration The object where the configuration will be stored
   * @param timeout The duration that the client should wait for the
   * configuration, if set to ros::Duration(0) will wait indefinetely
   * @return False if the timeout has happened
   */
  bool getCurrentConfiguration(
      ConfigType& configuration,
      const ros::Duration& timeout = ros::Duration(0)) {
    if (timeout == ros::Duration(0)) {
      ROS_INFO_ONCE("Waiting for configuration...");
      boost::mutex::scoped_lock lock(mutex_);
      while (!received_configuration_) {
        if (!ros::ok()) return false;
        cv_.wait(lock);
      }
    } else {
      ros::Time start_time = ros::Time::now();
      boost::mutex::scoped_lock lock(mutex_);
      while (!received_configuration_) {
        if (!ros::ok()) return false;
        ros::Duration time_left = timeout - (ros::Time::now() - start_time);
        if (time_left.toSec() <= 0.0) return false;
        cv_.wait_for(lock, boost::chrono::nanoseconds(time_left.toNSec()));
      }
    }
    configuration = latest_configuration_;
    return true;
  }
  /**
   * @brief getDefaultConfiguration Gets the latest default configuration from
   * the dynamic_reconfigure::Server
   * @param configuration The object where the configuration will be stored
   * @param timeout The duration that the client should wait for the
   * configuration, if set to ros::Duration(0) will wait indefinetely
   * @return False if the timeout has happened
   */
  bool getDefaultConfiguration(
      ConfigType& configuration,
      const ros::Duration& timeout = ros::Duration(0)) {
    ConfigDescription answer;
    if (getDescription(answer, timeout)) {
      configuration.__fromMessage__(answer.dflt);
      return true;
    } else {
      return false;
    }
  }
  /**
   * @brief getMinConfiguration Gets the latest minimum configuration from
   * the dynamic_reconfigure::Server
   * @param configuration The object where the configuration will be stored
   * @param timeout The duration that the client should wait for the
   * configuration, if set to ros::Duration(0) will wait indefinetely
   * @return False if the timeout has happened
   */
  bool getMinConfiguration(ConfigType& configuration,
                           const ros::Duration& timeout = ros::Duration(0)) {
    ConfigDescription answer;
    if (getDescription(answer, timeout)) {
      configuration.__fromMessage__(answer.min);
      return true;
    } else {
      return false;
    }
  }
  /**
   * @brief getMaxConfiguration Gets the latest maximum configuration from
   * the dynamic_reconfigure::Server
   * @param configuration The object where the configuration will be stored
   * @param timeout The duration that the client should wait for the
   * configuration, if set to ros::Duration(0) will wait indefinetely
   * @return False if the timeout has happened
   */
  bool getMaxConfiguration(ConfigType& configuration,
                           const ros::Duration& timeout = ros::Duration(0)) {
    ConfigDescription answer;
    if (getDescription(answer, timeout)) {
      configuration.__fromMessage__(answer.max);
      return true;
    } else {
      return false;
    }
  }
  /**
   * @brief getName Gets the name of the Dynamic Reconfigure Client
   * @return Copy of the member variable
   */
  std::string getName() const { return name_; }

 private:
  void configurationCallback(const dynamic_reconfigure::Config& configuration) {
    boost::mutex::scoped_lock lock(mutex_);
    dynamic_reconfigure::Config temp_config = configuration;
    received_configuration_ = true;
    latest_configuration_.__fromMessage__(temp_config);
    cv_.notify_all();

    if (config_callback_) {
      try {
        config_callback_(latest_configuration_);
      } catch (std::exception& e) {
        ROS_WARN("Configuration callback failed with exception %s", e.what());
      } catch (...) {
        ROS_WARN("Configuration callback failed with unprintable exception");
      }
    } else {
      ROS_DEBUG(
          "Unable to call Configuration callback because none was set.\n" \
          "See setConfigurationCallback");
    }
  }

  void descriptionCallback(
      const dynamic_reconfigure::ConfigDescription& description) {
    boost::mutex::scoped_lock lock(mutex_);
    received_description_ = true;
    latest_description_ = description;
    cv_.notify_all();

    if (description_callback_) {
      try {
        description_callback_(description);
      } catch (std::exception& e) {
        ROS_WARN("Description callback failed with exception %s", e.what());
      } catch (...) {
        ROS_WARN("Description callback failed with unprintable exception");
      }
    } else {
      ROS_DEBUG(
          "Unable to call Description callback because none was set.\n" \
          "See setDescriptionCallback");
    }
  }

  bool getDescription(ConfigDescription& configuration,
                      const ros::Duration& timeout) {
    if (timeout == ros::Duration(0)) {
      ROS_INFO_ONCE("Waiting for configuration...");
      boost::mutex::scoped_lock lock(mutex_);
      while (!received_description_) {
        if (!ros::ok()) return false;
        cv_.wait(lock);
      }
    } else {
      ros::Time start_time = ros::Time::now();
      boost::mutex::scoped_lock lock(mutex_);
      while (!received_description_) {
        if (!ros::ok()) return false;
        ros::Duration time_left = timeout - (ros::Time::now() - start_time);
        if (time_left.toSec() <= 0.0) return false;
        cv_.wait_for(lock, boost::chrono::nanoseconds(time_left.toNSec()));
      }
    }
    configuration = latest_description_;
    return true;
  }

  std::string name_;
  bool received_configuration_;
  ConfigType latest_configuration_;
  bool received_description_;
  dynamic_reconfigure::ConfigDescription latest_description_;
  boost::condition_variable cv_;
  boost::mutex mutex_;
  ros::NodeHandle nh_;
  ros::ServiceClient set_service_;
  ros::Subscriber descr_sub_;
  ros::Subscriber config_sub_;

  boost::function<void(const ConfigType&)> config_callback_;
  boost::function<void(const dynamic_reconfigure::ConfigDescription&)>
      description_callback_;
};
}

#endif  // __CLIENT_H__
