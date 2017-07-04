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

#ifndef ROSCPP_DISCOVERY_PARTICIPANT_H
#define ROSCPP_DISCOVERY_PARTICIPANT_H

#include <inttypes.h>
#include <functional>
#include <string>
#include <vector>
#include <mutex>

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>

#include <fastrtps/Domain.h>
#include <fastrtps/utils/eClock.h>

#include "MetaMessagePubSubTypes.h"
#include "MetaMessage.h"

namespace ros 
{

typedef std::function<void (const std::string&)> user_callback;

class Listener : public eprosima::fastrtps::SubscriberListener
{
public:
  Listener(const eprosima::fastrtps::rtps::GUID_t &guid) : _local_publisher_guid(guid) {}
  ~Listener() {}
  void onSubscriptionMatched(eprosima::fastrtps::Subscriber* /*sub*/,
                 eprosima::fastrtps::MatchingInfo& /*info*/) {}
  void onNewDataMessage(eprosima::fastrtps::Subscriber* sub)
  {
    eprosima::fastrtps::SampleInfo_t m_info;
    eprosima::fastrtps::SubscriberAttributes attr = sub->getAttributes();
    MetaMessage m;

    if (sub->takeNextData((void*)&m, &m_info) &&
          m_info.sampleKind == eprosima::fastrtps::ALIVE)
    {
      if (_local_publisher_guid == m_info.sample_identity.writer_guid())
      {
        return;
      }
      std::lock_guard<std::mutex> lock(_internal_mutex);
      if (_meta_callback)
      {
        _meta_callback(m.data());
      }
    }
  }

  bool register_callback(user_callback cb)
  {
    _meta_callback = cb;
    return true;
  }

private:
  eprosima::fastrtps::rtps::GUID_t _local_publisher_guid;
  std::mutex _internal_mutex;
  user_callback _meta_callback;
};

class Participant 
{
public:
  Participant(const std::string& name) :
    _inited(false), _name(name), _participant(nullptr),
    _meta_publisher(nullptr), _meta_subscriber(nullptr), _meta_listener(nullptr) {}

  ~Participant() 
  {
    if (_participant) 
    {
      Domain::removeParticipant(_participant);
    }
    if (_meta_listener) {
      delete _meta_listener;
    }
  }

  bool init(user_callback cb);
  bool init_py();
  void cache_msg(const std::string& msg);
  std::string read_msg();

  void send(const std::string& message) 
  {
    if (_meta_publisher) 
    {
      MetaMessage m;

      m.data(message);
      _meta_publisher->write((void*)&m);
    }
  }

private:
  bool _inited;
  std::string _name;
  eprosima::fastrtps::Participant* _participant;
  std::deque<std::string> _unread_msg;
  std::mutex _msg_lock;
  std::condition_variable _msg_cond;

  std::string _meta_topic_name;
  eprosima::fastrtps::Publisher * _meta_publisher;
  eprosima::fastrtps::Subscriber * _meta_subscriber;
  Listener* _meta_listener;
  eprosima::fastrtps::PublisherListener _pub_listener;
  ::MetaMessagePubSubType _type;
};

}

#endif
