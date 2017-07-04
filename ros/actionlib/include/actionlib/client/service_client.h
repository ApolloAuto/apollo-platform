/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef ACTIONLIB_CLIENT_SERVICE_CLIENT_H_
#define ACTIONLIB_CLIENT_SERVICE_CLIENT_H_
#include <actionlib/action_definition.h>
#include <actionlib/client/simple_action_client.h>

namespace actionlib {
  class ServiceClientImp {
    public:
      ServiceClientImp(){}
      virtual bool call(const void* goal, std::string goal_md5sum, void* result, std::string result_md5sum) = 0;
      virtual bool waitForServer(const ros::Duration& timeout) = 0;
      virtual bool isServerConnected() = 0;
      virtual ~ServiceClientImp(){}
  };

  class ServiceClient {
    public:
      ServiceClient(boost::shared_ptr<ServiceClientImp> client) : client_(client) {}

      template <class Goal, class Result>
      bool call(const Goal& goal, Result& result);

      bool waitForServer(const ros::Duration& timeout = ros::Duration(0,0));
      bool isServerConnected();

    private:
      boost::shared_ptr<ServiceClientImp> client_;
  };

  template <class ActionSpec>
  ServiceClient serviceClient(ros::NodeHandle n, std::string name);

  template <class ActionSpec>
  class ServiceClientImpT : public ServiceClientImp
  {
    public:
      ACTION_DEFINITION(ActionSpec);
      typedef ClientGoalHandle<ActionSpec> GoalHandleT;
      typedef SimpleActionClient<ActionSpec> SimpleActionClientT;

      ServiceClientImpT(ros::NodeHandle n, std::string name);

      bool call(const void* goal, std::string goal_md5sum, void* result, std::string result_md5sum);
      bool waitForServer(const ros::Duration& timeout);
      bool isServerConnected();

    private:
      boost::scoped_ptr<SimpleActionClientT> ac_;
      
  };
};

//include the implementation
#include <actionlib/client/service_client_imp.h>
#endif
