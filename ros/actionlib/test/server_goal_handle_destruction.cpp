/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

//! \author Eitan Marder-Eppstein

#include <actionlib/server/action_server.h>
#include <actionlib/TestAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

namespace actionlib
{

class ServerGoalHandleDestructionTester
{
public:
  typedef ServerGoalHandle<TestAction> GoalHandle;

  ServerGoalHandleDestructionTester();

  ros::NodeHandle nh_;
  ActionServer<TestAction>* as_;
  GoalHandle* gh_;

  ~ServerGoalHandleDestructionTester();
  void goalCallback(GoalHandle gh);
};

}

using namespace actionlib;

ServerGoalHandleDestructionTester::ServerGoalHandleDestructionTester()
{
  as_ = new ActionServer<TestAction>(nh_, "reference_action", false);
  as_->start();
  as_->registerGoalCallback(boost::bind(&ServerGoalHandleDestructionTester::goalCallback, this, _1));
  gh_ = new GoalHandle();

}

ServerGoalHandleDestructionTester::~ServerGoalHandleDestructionTester(){
  delete as_;
  gh_->setAccepted();
  delete gh_;
}

void ServerGoalHandleDestructionTester::goalCallback(GoalHandle gh)
{
  ROS_ERROR_NAMED("actionlib", "In callback");
  //assign to our stored goal handle
  *gh_ = gh;

  TestGoal goal = *gh.getGoal();

  switch (goal.goal)
  {
    case 1:
      gh.setAccepted();
      gh.setSucceeded(TestResult(), "The ref server has succeeded");
      break;
    case 2:
      gh.setAccepted();
      gh.setAborted(TestResult(), "The ref server has aborted");
      break;
    case 3:
      gh.setRejected(TestResult(), "The ref server has rejected");
      break;
    default:
      break;
  }

  ros::shutdown();
}

void spinner(){
  ros::spin();
}

TEST(ServerGoalHandleDestruction, destruction_test){
  boost::thread spin_thread(&spinner);

  ServerGoalHandleDestructionTester server;

  SimpleActionClient<TestAction> client("reference_action", true);

  ROS_ERROR_NAMED("actionlib", "Waiting for server");
  client.waitForServer();
  ROS_ERROR_NAMED("actionlib", "Done waiting for server");

  TestGoal goal;

  goal.goal = 1;
  client.sendGoal(goal);
  ROS_ERROR_NAMED("actionlib", "Sending goal");

  spin_thread.join();

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "ref_server");

  return RUN_ALL_TESTS();
}


