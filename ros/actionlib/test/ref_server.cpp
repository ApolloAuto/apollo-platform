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

//! \author Vijay Pradeep

#include <actionlib/server/action_server.h>
#include <actionlib/TestAction.h>
#include <ros/ros.h>
#include <cstdio>

namespace actionlib
{

class RefServer : public ActionServer<TestAction>
{
public:
  typedef ServerGoalHandle<TestAction> GoalHandle;

  RefServer(ros::NodeHandle& n, const std::string& name);

private:

  void goalCallback(GoalHandle gh);
  void cancelCallback(GoalHandle gh);

};

}

using namespace actionlib;

RefServer::RefServer(ros::NodeHandle& n, const std::string& name)
  : ActionServer<TestAction>(n, name,
                             boost::bind(&RefServer::goalCallback, this, _1),
                             boost::bind(&RefServer::cancelCallback, this, _1),
                             false)
{
  start();
  printf("Creating ActionServer [%s]\n", name.c_str());
}

void RefServer::goalCallback(GoalHandle gh)
{
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
}

void RefServer::cancelCallback(GoalHandle gh)
{


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ref_server");

  ros::NodeHandle nh;

  RefServer ref_server(nh, "reference_action");

  ros::spin();
}


