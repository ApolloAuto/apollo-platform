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

 Author: Aris Synodinos

 Handles sychronizing node state with the ActionServer and setting/getting
 configuration.

 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/TestConfig.h>

using namespace dynamic_reconfigure;
using namespace dynamic_reconfigure_test;

TestConfig CONFIG;
ConfigDescription DESCRIPTION;

void configurationCallback(const TestConfig& config) {
  ROS_INFO(
      "Reconfiguration happened: %i %f %s %s %i %i Group1: %i Group2: %f %s",
      config.int_, config.double_, config.str_.c_str(), config.mstr_.c_str(),
      (int)config.bool_, config.level, config.group1_int, config.group2_double,
      config.group2_string.c_str());
  CONFIG = config;
}

void descriptionCallback(const ConfigDescription& description) {
  ROS_INFO("Received description");
}

TEST(dynamic_reconfigure_simple_client, Constructor) {
  Client<TestConfig> client("/ref_server");
  client.setConfigurationCallback(&configurationCallback);
  client.setDescriptionCallback(&descriptionCallback);
}

TEST(dynamic_reconfigure_simple_client, getConfigs) {
  Client<TestConfig> client("/ref_server", &configurationCallback,
                            &descriptionCallback);
  EXPECT_TRUE(client.getCurrentConfiguration(CONFIG));
  EXPECT_TRUE(client.getDefaultConfiguration(CONFIG));
  EXPECT_EQ(0, CONFIG.int_);
  EXPECT_TRUE(client.getMinConfiguration(CONFIG));
  EXPECT_EQ(-10, CONFIG.int_);
  EXPECT_TRUE(client.getMaxConfiguration(CONFIG));
  EXPECT_EQ(10, CONFIG.int_);
}

TEST(dynamic_reconfigure_simple_client, setConfig) {
  ROS_INFO("Setting configuration");
  Client<TestConfig> client("/ref_server", &configurationCallback,
                            &descriptionCallback);
  TestConfig cfg = TestConfig::__getMax__();
  EXPECT_TRUE(client.setConfiguration(cfg));
  // int_ goes from -10 to +10
  cfg.int_ = -11;
  EXPECT_TRUE(client.setConfiguration(cfg));
  EXPECT_EQ(-10, cfg.int_);
  cfg.int_ = 11;
  EXPECT_TRUE(client.setConfiguration(cfg));
  EXPECT_EQ(10, cfg.int_);
}

TEST(dynamic_reconfigure_simple_client, setGetConfig) {
  ROS_INFO("Setting configuration");
  Client<TestConfig> client("/ref_server", &configurationCallback,
                            &descriptionCallback);
  TestConfig cfg = TestConfig::__getMax__();
  EXPECT_TRUE(client.setConfiguration(cfg));
  // int_ goes from -10 to +10
  cfg.int_ = -11;
  EXPECT_TRUE(client.setConfiguration(cfg));
  EXPECT_EQ(-10, cfg.int_);
  EXPECT_TRUE(client.getCurrentConfiguration(CONFIG));
  EXPECT_EQ(-10, CONFIG.int_);
  cfg.int_ = 11;
  EXPECT_TRUE(client.setConfiguration(cfg));
  EXPECT_TRUE(client.getCurrentConfiguration(CONFIG));
  EXPECT_EQ(10, CONFIG.int_);
  cfg.int_ = 5;
  EXPECT_TRUE(client.setConfiguration(cfg));
  EXPECT_TRUE(client.getCurrentConfiguration(CONFIG));
  EXPECT_EQ(5, CONFIG.int_);
}

TEST(dynamic_reconfigure_simple_client, multipleClients) {
  Client<TestConfig> client1("/ref_server", &configurationCallback);
  Client<TestConfig> client2("/ref_server", &configurationCallback);
  Client<TestConfig> client3("/ref_server", &configurationCallback);
  client3.setConfiguration(TestConfig::__getDefault__());
  ros::Duration(0.2).sleep();
  EXPECT_EQ(0, CONFIG.int_);
  client1.setConfiguration(TestConfig::__getMin__());
  ros::Duration(0.2).sleep();
  EXPECT_EQ(-10, CONFIG.int_);
  client2.setConfiguration(TestConfig::__getMax__());
  ros::Duration(0.2).sleep();
  EXPECT_EQ(10, CONFIG.int_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_reconfigure_client_test");
  testing::InitGoogleTest(&argc, argv);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  return RUN_ALL_TESTS();
}
