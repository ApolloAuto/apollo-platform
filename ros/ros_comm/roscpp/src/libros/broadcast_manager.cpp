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

#include "ros/xmlrpc_manager.h"
#include "ros/broadcast_manager.h" 
#include "ros/network.h"
#include "ros/assert.h"
#include "ros/common.h"
#include "ros/file_log.h"
#include "ros/this_node.h"
#include "ros/io.h"
#include "ros/init.h"
#include "ros/time.h"

#include <string>
#include <cstdlib>
#include <sstream>
#include <chrono>

namespace ros 
{

BroadcastManagerPtr g_br_manager;
std::mutex g_br_manager_mutex;

const BroadcastManagerPtr& BroadcastManager::instance() 
{
  if (!g_br_manager) 
  {
    std::lock_guard<std::mutex> lg(g_br_manager_mutex);

    if (!g_br_manager) 
    {
      g_br_manager.reset(new BroadcastManager);
    }
  }

  return g_br_manager;
}

BroadcastManager::BroadcastManager() : shutting_down_(false) 
{
  xmlrpc_manager_ = XMLRPCManager::instance();
  topic_manager_ = TopicManager::instance();
  initCallbacks();

  node_timestamp_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  part_.reset(new Participant(this_node::getName()));
  static auto f = [this] (const std::string& msg) {this->commonCallback(msg);};  
  if (!part_->init(f)) 
  {
    ROS_ERROR_STREAM("+++BroadcastManager+++ Init Participant failed.");
  } 
}

void BroadcastManager::start() 
{
  shutting_down_ = false;
  usleep(1000000);
  registerNode();
}

void BroadcastManager::initCallbacks() 
{
  REGISTRY(registerPublisher, registerPublisher);
  REGISTRY(unregisterPublisher, unregisterPublisher);
  REGISTRY(registerSubscriber, registerSubscriber);
  REGISTRY(unregisterSubscriber, unregisterSubscriber);
  REGISTRY(registerService, registerService);
  REGISTRY(unregisterService, unregisterService);
}

BroadcastManager::~BroadcastManager() 
{
  shutdown();
}

void BroadcastManager::registerNode() 
{
  MsgInfo req = generateMessage("registerNode");
  req.put(NODE_TIME, std::to_string(node_timestamp_));
  sendMessage(req);
}

void BroadcastManager::sendMessage(const MsgInfo& pt) 
{
  std::stringstream ss;
  write_json(ss, pt);
  sendMessage(ss.str());
}

void BroadcastManager::sendMessage(const std::string& msg) 
{
  ROS_DEBUG_STREAM("[BroadcastManager]Send msg: " << msg);
  part_->send(msg);
}

void BroadcastManager::shutdown() 
{
  if (isShuttingDown()) 
  {
    return;
  }
  shutting_down_ = true;
}

PubInfo BroadcastManager::getPubs(const std::string& topic_name) 
{
  return pub_cache_[topic_name]; 
}

SubInfo BroadcastManager::getSubs(const std::string& topic_name) 
{
  return sub_cache_[topic_name]; 
}

void BroadcastManager::publisherUpdate(const std::string& topic_name) 
{
  std::vector<std::string> pubs(pub_cache_[topic_name].begin(), pub_cache_[topic_name].end());
  topic_manager_->pubUpdate(topic_name, pubs);
}

/********* message callbacks **********/
void BroadcastManager::registerPublisherCallback(const MsgInfo& result) 
{
  std::string topic_name = result.get<std::string>(TOPIC_NAME);
  std::string pub_uri = result.get<std::string>(XMLRPC_URI);
  std::string data_type = result.get<std::string>(TOPIC_TYPE);
  pub_cache_[topic_name].insert(pub_uri);
  topic_cache_.emplace_back(topic_name, data_type);
  publisherUpdate(topic_name);
}

void BroadcastManager::unregisterPublisherCallback(const MsgInfo& result) 
{
  std::string topic_name = result.get<std::string>(TOPIC_NAME);
  std::string uri = result.get<std::string>(XMLRPC_URI);

  auto it = pub_cache_[topic_name].find(uri);
  if (it != pub_cache_[topic_name].end()) 
  {
    pub_cache_[topic_name].erase(it);
  }
  publisherUpdate(topic_name);
}

void BroadcastManager::registerSubscriberCallback(const MsgInfo& result) 
{
  std::string topic_name = result.get<std::string>(TOPIC_NAME);
  std::string sub_uri = result.get<std::string>(XMLRPC_URI);
  sub_cache_[topic_name].insert(sub_uri);
}

void BroadcastManager::unregisterSubscriberCallback(const MsgInfo& result) 
{
  std::string topic = result.get<std::string>(TOPIC_NAME);
  std::string uri = result.get<std::string>(XMLRPC_URI);
  auto it = sub_cache_[topic].find(uri);
  if (it != sub_cache_[topic].end()) 
  {
    sub_cache_[topic].erase(it);
  }
}

void BroadcastManager::registerServiceCallback(const MsgInfo& result) 
{
  std::string service_name = result.get<std::string>(SERVICE_NAME);
  std::string uri = result.get<std::string>(SERVICE_URI);

  ServiceInfo si;
  si.uri = uri;
  si.is_owner = false;
  service_cache_[service_name] = std::move(si);
}


void BroadcastManager::unregisterServiceCallback(const MsgInfo& result) 
{
  std::string srv_name = result.get<std::string>(SERVICE_NAME);
  std::string uri = result.get<std::string>(SERVICE_URI);

  auto it = service_cache_.find(srv_name);
  if (it != service_cache_.end() && it->second.uri == uri) 
  {
    service_cache_.erase(it);
  }
}

void BroadcastManager::commonCallback(const std::string& msg) 
{
  MsgInfo result;
  std::stringstream ss(msg);
  try 
  {
    read_json(ss, result);
  } 
  catch (std::exception const& e) 
  {
    ROS_ERROR_STREAM("Failed to parse message: " << e.what());
    return;
  }

  if (result.get<std::string>(NODE_NAME) == this_node::getName()) 
  { 
    if (result.get<std::string>(REQUEST_TYPE) == "registerNode"
        && result.get<std::string>(XMLRPC_URI) != xmlrpc_manager_->getServerURI()
        && std::stoul(result.get<std::string>(NODE_TIME)) > node_timestamp_) 
    {
      ROS_ERROR_STREAM("Shutdown! Another node registered with same name");
      requestShutdown();
    }
    return;
  }

  ROS_DEBUG_STREAM("[BroadcastManager] Recv msg: " << msg.c_str());
  auto it = functions_.find(result.get<std::string>(REQUEST_TYPE));
  if (it != functions_.end()) 
  {
    it->second(result);
  } 
  else 
  {
    ROS_DEBUG_STREAM("invalid request type: " << result.get<std::string>(REQUEST_TYPE).c_str());
  }
}


/************************* Interfaces ********************************/
void BroadcastManager::registerPublisher(const std::string& topic, 
                                   const std::string& datatype, 
		                           const std::string& uri) 
{
  MsgInfo req = generateMessage("registerPublisher");
  req.put(TOPIC_NAME, topic);
  req.put(TOPIC_TYPE, datatype);
  sendMessage(req);

  TopicInfo ti;
  ti.type = datatype;
  advertised_topics_[topic] = ti;
  pub_cache_[topic].insert(uri);
}


void BroadcastManager::unregisterPublisher(const std::string& topic_name, const std::string& uri) 
{
  (void)uri;
  MsgInfo req = generateMessage("unregisterPublisher");
  req.put(TOPIC_NAME, topic_name);
  req.put(TOPIC_URI, uri);
  sendMessage(req);

  auto it = pub_cache_[topic_name].find(req.get<std::string>(XMLRPC_URI));
  if (it != pub_cache_[topic_name].end()) 
  {
    pub_cache_[topic_name].erase(it);
  }
}

void BroadcastManager::registerSubscriber(const std::string& topic, const std::string& type, const std::string& uri) 
{
  (void)uri;
  MsgInfo req = generateMessage("registerSubscriber");
  req.put(TOPIC_NAME, topic);
  req.put(TOPIC_TYPE, type);
  sendMessage(req);

  sub_cache_[topic].insert(req.get<std::string>(XMLRPC_URI));
}

void BroadcastManager::unregisterSubscriber(const std::string& topic, const std::string& uri) 
{
  (void)uri;
  MsgInfo req = generateMessage("unregisterSubscriber");
  req.put(TOPIC_NAME, topic);
  req.put(TOPIC_URI, uri);
  sendMessage(req);
  auto it = sub_cache_[topic].find(req.get<std::string>(XMLRPC_URI));
  if (it != sub_cache_[topic].end()) 
  {
    sub_cache_[topic].erase(it);
  }
}

void BroadcastManager::registerService(const std::string& node_name, 
                                 const std::string& srv_name,
                                 const std::string& uri_buf,
                                 const std::string& uri) 
{
  (void)node_name;
  (void)uri;
  MsgInfo req = generateMessage("registerService");
  req.put(SERVICE_NAME, srv_name);
  req.put(SERVICE_URI, uri_buf);
  sendMessage(req);

  ServiceInfo si;
  si.uri = uri_buf;
  si.is_owner = true;
  service_cache_[srv_name] = std::move(si);
}

void BroadcastManager::unregisterService(const std::string& srv_name, const std::string& uri) 
{
  MsgInfo req = generateMessage("unregisterService");
  req.put(SERVICE_NAME, srv_name);
  req.put(SERVICE_URI, uri);
  sendMessage(req);
}

bool BroadcastManager::lookupService(const std::string& service_name, std::string& service_uri, int timeout) 
{
  (void)timeout;
  usleep(1000);
  auto it = service_cache_.find(service_name);
  if (it != service_cache_.end()) 
  {
    service_uri = it->second.uri;
  } 
  else 
  {
    return false;
  }
  return true;
}

MsgInfo BroadcastManager::generateMessage(const std::string& request_type) 
{
  uint64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); 
  return generateMessage(request_type, timestamp);
}

MsgInfo BroadcastManager::generateMessage(const std::string& request_type, uint64_t timestamp) 
{
  MsgInfo req;
  req.put(NODE_NAME, this_node::getName());
  req.put(REQUEST_TYPE, request_type);
  req.put(TIMESTAMP, std::to_string(timestamp));
  req.put(XMLRPC_URI, xmlrpc_manager_->getServerURI());
  return req;
}

void BroadcastManager::getNodes(V_string& nodes) 
{
  for (auto& pub: pub_cache_) 
  {
    nodes.push_back(pub.first);
  }
}

void BroadcastManager::getTopics(master::V_TopicInfo& topics) 
{
  topics.clear();
  topics.insert(topics.begin(), topic_cache_.begin(), topic_cache_.end());
}

void BroadcastManager::getTopicTypes(master::V_TopicInfo& topics) 
{
  return getTopics(topics);
}


} // namespace ros
