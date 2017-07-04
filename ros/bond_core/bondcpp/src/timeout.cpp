/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include <bondcpp/timeout.h>

namespace bond {

Timeout::Timeout(const ros::Duration &d,
                 boost::function<void(void)> on_timeout)
  : duration_(d.sec, d.nsec), on_timeout_(on_timeout)
{
}

Timeout::Timeout(const ros::WallDuration &d,
                 boost::function<void(void)> on_timeout)
  : duration_(d), on_timeout_(on_timeout)
{
}

Timeout::~Timeout()
{
  timer_.stop();
}

void Timeout::setDuration(const ros::Duration &d)
{
  duration_ = ros::WallDuration(d.sec, d.nsec);
}

void Timeout::setDuration(const ros::WallDuration &d)
{
  duration_ = d;
}


void Timeout::reset()
{
  timer_.stop();
  timer_ = nh_.createWallTimer(duration_, &Timeout::timerCallback, this, true);
  deadline_ = ros::WallTime::now() + duration_;
}

void Timeout::cancel()
{
  timer_.stop();
}

ros::WallDuration Timeout::left()
{
  return std::max(ros::WallDuration(0.0), deadline_ - ros::WallTime::now());
}

void Timeout::timerCallback(const ros::WallTimerEvent &)
{
  if (on_timeout_)
    on_timeout_();
}

}
