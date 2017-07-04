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

// Derived from excercise_simple_server.py

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <actionlib/TestRequestAction.h>
#include <actionlib/TestRequestGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>


#define EXPECT_STATE(expected_state) EXPECT_TRUE( ac_.getState() == SimpleClientGoalState::expected_state) \
  << "Expected [" << #expected_state << "], but goal state is [" << ac_.getState().toString() << "]";


using namespace actionlib;
using namespace actionlib_msgs;

class SimpleClientFixture : public testing::Test
{
public:
  SimpleClientFixture() : ac_("test_request_action"), default_wait_(60.0)  {  }

protected:
  virtual void SetUp()
  {
    // Make sure that the server comes up
    ASSERT_TRUE( ac_.waitForServer(ros::Duration(10.0)) );
  }

  SimpleActionClient<TestRequestAction> ac_;
  ros::Duration default_wait_;
};

TEST_F(SimpleClientFixture, just_succeed)
{
  TestRequestGoal goal;
  goal.terminate_status = TestRequestGoal::TERMINATE_SUCCESS;
  goal.the_result = 42;
  ac_.sendGoal(goal);
  ac_.waitForResult(default_wait_);
  EXPECT_STATE(SUCCEEDED);
  EXPECT_EQ(42, ac_.getResult()->the_result);
}

TEST_F(SimpleClientFixture, just_abort)
{
  TestRequestGoal goal;
  goal.terminate_status = TestRequestGoal::TERMINATE_ABORTED;
  goal.the_result = 42;
  ac_.sendGoal(goal);
  ac_.waitForResult(default_wait_);
  EXPECT_STATE(ABORTED);
  EXPECT_EQ(42, ac_.getResult()->the_result);
}

TEST_F(SimpleClientFixture, just_preempt)
{
  TestRequestGoal goal;
  goal.terminate_status = TestRequestGoal::TERMINATE_SUCCESS;
  goal.delay_terminate = ros::Duration(1000);
  goal.the_result = 42;
  ac_.sendGoal(goal);

  // Sleep for 10 seconds or until we hear back from the action server
  for (unsigned int i=0; i < 100; i++)
  {
    ROS_DEBUG_NAMED("actionlib", "Waiting for Server Response");
    if (ac_.getState() != SimpleClientGoalState::PENDING)
      break;
    ros::Duration(0.1).sleep();
  }

  ac_.cancelGoal();
  ac_.waitForResult(default_wait_);
  EXPECT_STATE(PREEMPTED);
  EXPECT_EQ(42, ac_.getResult()->the_result);
}

TEST_F(SimpleClientFixture, drop)
{
  TestRequestGoal goal;
  goal.terminate_status = TestRequestGoal::TERMINATE_DROP;
  goal.the_result = 42;
  ac_.sendGoal(goal);
  ac_.waitForResult(default_wait_);
  EXPECT_STATE(ABORTED);
  EXPECT_EQ(0, ac_.getResult()->the_result);
}

TEST_F(SimpleClientFixture, exception)
{
  TestRequestGoal goal;
  goal.terminate_status = TestRequestGoal::TERMINATE_EXCEPTION;
  goal.the_result = 42;
  ac_.sendGoal(goal);
  ac_.waitForResult(default_wait_);
  EXPECT_STATE(ABORTED);
  EXPECT_EQ(0, ac_.getResult()->the_result);
}

TEST_F(SimpleClientFixture, ignore_cancel_and_succeed)
{
  TestRequestGoal goal;
  goal.terminate_status = TestRequestGoal::TERMINATE_SUCCESS;
  goal.delay_terminate = ros::Duration(2.0);
  goal.ignore_cancel = true;
  goal.the_result = 42;
  ac_.sendGoal(goal);

  // Sleep for 10 seconds or until we hear back from the action server
  for (unsigned int i=0; i < 100; i++)
  {
    ROS_DEBUG_NAMED("actionlib", "Waiting for Server Response");
    if (ac_.getState() != SimpleClientGoalState::PENDING)
      break;
    ros::Duration(0.1).sleep();
  }

  ac_.cancelGoal();
  ac_.waitForResult(default_wait_ + default_wait_);
  EXPECT_STATE(SUCCEEDED);
  EXPECT_EQ(42, ac_.getResult()->the_result);
}

TEST_F(SimpleClientFixture, lose)
{
  TestRequestGoal goal;
  goal.terminate_status = TestRequestGoal::TERMINATE_LOSE;
  goal.the_result = 42;
  ac_.sendGoal(goal);
  ac_.waitForResult(default_wait_);
  EXPECT_STATE(LOST);
}

void spinThread()
{
  ros::NodeHandle nh;
  ros::spin();
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "simple_client_test");

  boost::thread spin_thread(&spinThread);

  int result = RUN_ALL_TESTS();

  ros::shutdown();

  spin_thread.join();

  return result;
}
