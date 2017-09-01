/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#ifndef ROSLIB_PROTOBUFFER_TRAITS_H
#define ROSLIB_PROTOBUFFER_TRAITS_H

#include "message_forward.h"
#include "message_traits.h"

#include <ros/time.h>
#include <ros/md5.h>

#include <string>
#include <map>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <typeinfo>

#include <google/protobuf/message.h>
#include <google/protobuf/descriptor.h>

namespace ros
{
namespace message_traits
{
//protobuffer trait
static std::map<std::string, std::string> type_md5_map;

template<typename T>
struct DataType<T, typename boost::enable_if<boost::is_base_of< ::google::protobuf::Message, T> >::type>
{
  static const char* value() 
  { 
    static std::string data_type = "";
    data_type = "pb_msgs/" + T::descriptor()->name();
    return data_type.c_str(); 
  }
  static const char* value(const T&) { return value(); }
};

template<typename T>
struct MD5Sum<T, typename boost::enable_if<boost::is_base_of< ::google::protobuf::Message, T> >::type>
{
  static const char* value() 
  {
    std::string data_type(DataType<T>::value());
    if (type_md5_map.count(data_type) == 0) 
    {
      type_md5_map[data_type] = ros::md5::MD5(data_type).toStr();
    }
    return type_md5_map[data_type].c_str();
  }
  static const char* value(const T&) { return value(); }
};

template<typename T>
struct Definition<T, typename boost::enable_if<boost::is_base_of< ::google::protobuf::Message, T> >::type>
{
  static const char* value() { return "protobuf"; }
  static const char* value(const T&) { return "protobuf"; }
};

template<typename T>
struct HasHeader<T, typename boost::enable_if<boost::is_base_of< ::google::protobuf::Message, T> >::type>
  : FalseType
{ };

template<typename T>
struct HasHeader<T const, typename boost::enable_if<boost::is_base_of< ::google::protobuf::Message, T> >::type>
  : FalseType
{ };

template<typename T>
struct IsFixedSize<T, typename boost::enable_if<boost::is_base_of< ::google::protobuf::Message, T> >::type>
  : FalseType
{ };

template<typename T>
struct IsFixedSize<T const, typename boost::enable_if<boost::is_base_of< ::google::protobuf::Message, T> >::type>
  : FalseType
{ };

template<typename T>
struct IsMessage<T, typename boost::enable_if<boost::is_base_of< ::google::protobuf::Message, T> >::type>
  : TrueType
{ };

template<typename T>
struct IsMessage<T const, typename boost::enable_if<boost::is_base_of< ::google::protobuf::Message, T> >::type>
  : TrueType
{ };

} // namespace message_traits
} // namespace ros

#endif // ROSLIB_MESSAGE_TRAITS_H
