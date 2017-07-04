/*
 * Copyright (c) 2008, Willow Garage, Inc.
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


/** \author Tully Foote */


#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace tf2_ros {

StaticTransformBroadcaster::StaticTransformBroadcaster()
{
  publisher_ = node_.advertise<tf2_msgs::TFMessage>("/tf_static", 100, true);
};

void StaticTransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped & msgtf)
{
  std::vector<geometry_msgs::TransformStamped> v1;
  v1.push_back(msgtf);
  sendTransform(v1);
}


void StaticTransformBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped> & msgtf)
{
  for (std::vector<geometry_msgs::TransformStamped>::const_iterator it_in = msgtf.begin(); it_in != msgtf.end(); ++it_in)
  {
    bool match_found = false;
    for (std::vector<geometry_msgs::TransformStamped>::iterator it_msg = net_message_.transforms.begin(); it_msg != net_message_.transforms.end(); ++it_msg)
    {
      if (it_in->child_frame_id == it_msg->child_frame_id)
      {
        *it_msg = *it_in;
        match_found = true;
        break;
      }
    }
    if (! match_found)
      net_message_.transforms.push_back(*it_in);
  }

  publisher_.publish(net_message_);
}


}


