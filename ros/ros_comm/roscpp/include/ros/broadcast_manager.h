/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, The Apollo Authors.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

#ifndef ROSCPP_BROADCAST_MANAGER_H
#define ROSCPP_BROADCAST_MANAGER_H

#define BOOST_SPIRIT_THREADSAFE // must define before include boost libs

#include <condition_variable> 
#include <functional>
#include <future>
#include <queue>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <set>
#include <thread>
#include <vector>
#include <chrono>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "common.h"
#include "XmlRpc.h"
#include "ros/time.h"
#include "ros/master.h"
#include "ros/topic_manager.h"
#include "ros/xmlrpc_manager.h"
#include "discovery/participant.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define REGISTRY(name, method) \
  functions_[#name] = CallbackFunc([this] (const MsgInfo& result) \
      {this->method##Callback(result);});

namespace ros 
{

class BroadcastManager;

typedef boost::property_tree::ptree MsgInfo;
typedef std::set<std::string> PubInfo;
typedef std::set<std::string> SubInfo;
typedef std::function<void (const MsgInfo&)> CallbackFunc;
typedef std::shared_ptr<BroadcastManager> BroadcastManagerPtr;

struct TopicInfo 
{
  std::string type;
  std::string uri;
};

struct ServiceInfo 
{
  std::string uri;
  bool is_owner;
};

class ROSCPP_DECL BroadcastManager 
{

public:
  BroadcastManager();
  ~BroadcastManager();
  static const BroadcastManagerPtr& instance();

  void start();
  void shutdown();

  void registerPublisher(const std::string& topic, const std::string& datatype, const std::string& uri);
  void unregisterPublisher(const std::string& pub_name, const std::string& uri);

  void registerSubscriber(const std::string& topic, const std::string& type, const std::string& uri);
  void unregisterSubscriber(const std::string& sub_name, const std::string& uri);

  void registerService(const std::string& node_name, const std::string& srv_name,
                const std::string& uri_buf, const std::string& uri);
  void unregisterService(const std::string& service, const std::string& uri);
  bool lookupService(const std::string& srv_name, std::string& serv_uri, int timeout = 2000);

  PubInfo getPubs(const std::string& topic_name);
  SubInfo getSubs(const std::string& topic_name);

  void publisherUpdate(const std::string& topic_name);
  void getTopics(master::V_TopicInfo& topics);
  void getTopicTypes(master::V_TopicInfo& topics);
  void getNodes(V_string& nodes);
  void commonCallback(const std::string& result);

private:
  //callbacks for broadcast msg
  void registerPublisherCallback(const MsgInfo& result);
  void unregisterPublisherCallback(const MsgInfo& result);
  void registerSubscriberCallback(const MsgInfo& result);
  void unregisterSubscriberCallback(const MsgInfo& result);
  void registerServiceCallback(const MsgInfo& result);
  void unregisterServiceCallback(const MsgInfo& result);
  void noneCallback(const MsgInfo& result) 
  {
    (void)result;
  };

  void registerNode();
  void serviceUpdate(const std::string& name, const std::string& uri);

  MsgInfo generateMessage(const std::string& request_type);
  MsgInfo generateMessage(const std::string& request_type, uint64_t timestamp);

  void sendMessage(const MsgInfo& msg);
  void sendMessage(const std::string& msg);

  bool shutting_down_;

  void initCallbacks();

  bool isShuttingDown() {
    return shutting_down_;
  }

  TopicManagerPtr topic_manager_;
  XMLRPCManagerPtr xmlrpc_manager_;
  master::V_TopicInfo topic_cache_;
  std::map<std::string, PubInfo> pub_cache_;
  std::map<std::string, SubInfo> sub_cache_;
  std::map<std::string, ServiceInfo> service_cache_;
  std::map<std::string, TopicInfo> advertised_topics_;
  std::map<std::string, CallbackFunc> functions_;

  std::unique_ptr<Participant> part_;
  uint64_t node_timestamp_;

  const std::string NODE_NAME = "node_name";
  const std::string TIMESTAMP = "timestamp";
  const std::string XMLRPC_URI = "xmlrpc_uri";
  const std::string REQUEST_TYPE = "request_type";

  const std::string NODE_TIME = "node_time";

  const std::string TOPIC_NAME = "topic_name";
  const std::string TOPIC_TYPE = "topic_type";
  const std::string TOPIC_URI = "topic_uri";

  const std::string SERVICE_NAME = "service_name";
  const std::string SERVICE_TYPE = "service_type";
  const std::string SERVICE_URI = "service_uri";
};
} // namespace ros
#endif
