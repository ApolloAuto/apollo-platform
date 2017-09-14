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

#include "discovery/participant.h"
#include "ros/platform.h"
#include <string>
#include <vector>

namespace ros 
{

bool Participant::init_py() 
{
  auto cb = [this] (const std::string& msg) {this->cache_msg(msg);};
  init(cb);
  return true;
}

void Participant::cache_msg(const std::string& msg) 
{
  {
    std::lock_guard<std::mutex> lg(_msg_lock);
    _unread_msg.push_back(msg);
  }
  _msg_cond.notify_one();
}

std::string Participant::read_msg() 
{
  std::string msg = "";
  std::unique_lock<std::mutex> ul(_msg_lock);

  if (_unread_msg.size() > 0) 
  {
     msg = std::move(_unread_msg.front());
    _unread_msg.pop_front();
    return msg;
  }

  _msg_cond.wait(ul, [this]{return !this->_unread_msg.empty();});
  if (_unread_msg.size() > 0) 
  {
     msg = std::move(_unread_msg.front());
    _unread_msg.pop_front();
  }

  return msg;
}

bool Participant::init(user_callback cb) 
{
  if (_inited) 
  {
    return false;
  }

  std::string topic("ros_meta");
  std::string data_type("MetaMessage");

  int domain_id = 50000;
  const char *val = ::getenv("ROS_DOMAIN_ID");
  if (val != NULL) 
  {
    try 
    {
      domain_id = std::stoi(val); 
    } 
    catch (std::exception const &e) 
    {
      //using default domain_id.
    }
  }

  eprosima::fastrtps::ParticipantAttributes participant_param;
  participant_param.rtps.defaultSendPort = 50000;
  participant_param.rtps.use_IP6_to_send = false;
  participant_param.rtps.builtin.use_SIMPLE_RTPSParticipantDiscoveryProtocol = true;
  participant_param.rtps.builtin.use_SIMPLE_EndpointDiscoveryProtocol = true;
  participant_param.rtps.builtin.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter = true;
  participant_param.rtps.builtin.m_simpleEDP.use_PublicationWriterANDSubscriptionReader = true;
  participant_param.rtps.builtin.domainId = domain_id;
  participant_param.rtps.builtin.leaseDuration = c_TimeInfinite;
  participant_param.rtps.builtin.leaseDuration_announcementperiod.seconds = 3;
  participant_param.rtps.setName(_name.c_str());

  // set unicast ip
  std::string ip_env;
  if (get_environment_variable(ip_env, "ROS_IP")) 
  {
    if (ip_env.size() == 0)
    {
      std::cerr << "invalid ROS_IP (an empty string)" << std::endl;
    }

    eprosima::fastrtps::Locator_t locator;
    locator.port = 0; //RTPS
    locator.set_IP4_address(ip_env);
    locator.kind = LOCATOR_KIND_UDPv4;

    participant_param.rtps.defaultUnicastLocatorList.push_back(locator);
    participant_param.rtps.defaultOutLocatorList.push_back(locator);
    participant_param.rtps.builtin.metatrafficUnicastLocatorList.push_back(locator);

    locator.set_IP4_address(239, 255, 0, 1);
    participant_param.rtps.builtin.metatrafficMulticastLocatorList.push_back(locator);
  }

  _participant = Domain::createParticipant(participant_param);
  if (_participant == nullptr) 
  {
    std::cerr << "Create participant failed." << std::endl;
    return false;
  }

  Domain::registerType(_participant, &_type);

  //Create publisher
  eprosima::fastrtps::PublisherAttributes pub_param;
  pub_param.topic.topicName = topic;
  pub_param.topic.topicDataType = data_type;
  pub_param.historyMemoryPolicy = DYNAMIC_RESERVE_MEMORY_MODE;

  pub_param.topic.topicKind = NO_KEY;
  pub_param.topic.historyQos.kind = KEEP_ALL_HISTORY_QOS;
  pub_param.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
  pub_param.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
  pub_param.topic.historyQos.depth = 5000;
  pub_param.topic.resourceLimitsQos.max_samples = 10000;

  pub_param.times.heartbeatPeriod.seconds = 2;
  pub_param.times.heartbeatPeriod.fraction = 0;

  _meta_publisher = Domain::createPublisher(_participant, pub_param, &_pub_listener);
  if (_meta_publisher == nullptr) 
  {
    Domain::removeParticipant(_participant);
    _participant = nullptr;
    std::cerr << "Create publisher failed." << std::endl;
    return false;
  }

  // create listener
  eprosima::fastrtps::rtps::GUID_t guid = _meta_publisher->getGuid();

  _meta_listener = new Listener(guid);
  if (!_meta_listener) {
    Domain::removeParticipant(_participant);
    _participant = nullptr;
    std::cerr << "Create subsciber listener failed." << std::endl;
    return false;
  }

  _meta_listener->register_callback(cb);
  eprosima::fastrtps::SubscriberAttributes sub_param;
  sub_param.topic.topicName = topic;
  sub_param.topic.topicDataType = data_type;
  sub_param.historyMemoryPolicy = DYNAMIC_RESERVE_MEMORY_MODE;

  sub_param.topic.topicKind = NO_KEY;
  sub_param.topic.historyQos.kind = KEEP_ALL_HISTORY_QOS;
  sub_param.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
  sub_param.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
  sub_param.topic.historyQos.depth = 5000;
  sub_param.topic.resourceLimitsQos.max_samples = 10000;

  _meta_subscriber = Domain::createSubscriber(_participant, sub_param, _meta_listener);
  if (_meta_subscriber == nullptr) 
  {
    Domain::removeParticipant(_participant);
    _participant = nullptr;
    delete _meta_listener;
    _meta_listener = nullptr;
    std::cerr << "Create subsciber failed." << std::endl;
    return false;
  }

  return true;
}
}

