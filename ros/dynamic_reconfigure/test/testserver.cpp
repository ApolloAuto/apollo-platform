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

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/TestConfig.h>

void callback(dynamic_reconfigure::Server<dynamic_reconfigure::TestConfig>& srv, dynamic_reconfigure::TestConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %i %f %s %i %i Group1:%i Group2: %f %s", config.int_, config.double_, config.str_.c_str(), (int) config.bool_, config.level, config.group1_int, config.group2_double, config.group2_string.c_str());

  config.int_ |= 1;
  config.double_ = -config.double_;
  config.str_ += "A";
  config.bool_ = !config.bool_;
  config.level = level;
  dynamic_reconfigure::TestConfig max;
  srv.getConfigMax(max);
  max.int_ = max.int_ + 1;
  srv.setConfigMax(max);
  ROS_INFO("TEST");
  ROS_INFO("Group 2 requested to be set to %d", config.groups.group_one.group2.state);
  ROS_INFO("group2_doube requested to be set to %f", config.groups.group_one.group2.group2_double);

  ROS_INFO("Reconfigured to     : %i %f %s %i %i", config.int_, config.double_, config.str_.c_str(), (int) config.bool_, config.level);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_reconfigure_test_server");

  dynamic_reconfigure::Server<dynamic_reconfigure::TestConfig> srvs;
  dynamic_reconfigure::Server<dynamic_reconfigure::TestConfig>::CallbackType f = boost::bind(&callback, boost::ref(srvs),_1, _2);
  srvs.setCallback(f);
  ROS_INFO("Constants are: %i %f %s %i", dynamic_reconfigure::Test_int_const, dynamic_reconfigure::Test_double_const, dynamic_reconfigure::Test_str_const, (int) dynamic_reconfigure::Test_bool_const);
  ROS_INFO("Starting to spin...");
  ros::spin();
  return 0;
}
