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
#ifndef ACTIONLIB_CLIENT_SERVICE_CLIENT_IMP_H_
#define ACTIONLIB_CLIENT_SERVICE_CLIENT_IMP_H_
namespace actionlib {
  template <class ActionSpec>
    ServiceClientImpT<ActionSpec>::ServiceClientImpT(ros::NodeHandle n, std::string name){
      ac_.reset(new SimpleActionClientT(n, name, true));
    }

  template <class ActionSpec>
    bool ServiceClientImpT<ActionSpec>::waitForServer(const ros::Duration& timeout){
      return ac_->waitForServer(timeout);
    }

  template <class ActionSpec>
    bool ServiceClientImpT<ActionSpec>::isServerConnected(){
      return ac_->isServerConnected();
    }

  template <class ActionSpec>
    bool ServiceClientImpT<ActionSpec>::call(const void* goal, std::string goal_md5sum, 
                                             void* result, std::string result_md5sum)
  {
      //ok... we need to static cast the goal message and result message
      const Goal* goal_c = static_cast<const Goal*>(goal);
      Result* result_c = static_cast<Result*>(result);

      //now we need to check that the md5sums are correct
      namespace mt = ros::message_traits;

      if(strcmp(mt::md5sum(*goal_c), goal_md5sum.c_str()) || strcmp(mt::md5sum(*result_c), result_md5sum.c_str()))
      {
        ROS_ERROR_NAMED("actionlib", "Incorrect md5Sums for goal and result types");
        return false;
      }

      if(!ac_->isServerConnected()){
        ROS_ERROR_NAMED("actionlib", "Attempting to make a service call when the server isn't actually connected to the client.");
        return false;
      }

      ac_->sendGoalAndWait(*goal_c);
      if(ac_->getState() == SimpleClientGoalState::SUCCEEDED){
        (*result_c) = *(ac_->getResult());
        return true;
      }

      return false;
    }

  //****** ServiceClient *******************
  template <class Goal, class Result>
  bool ServiceClient::call(const Goal& goal, Result& result){
    namespace mt = ros::message_traits;
    return client_->call(&goal, mt::md5sum(goal), &result, mt::md5sum(result));
  }

  bool ServiceClient::waitForServer(const ros::Duration& timeout){
    return client_->waitForServer(timeout);
  }

  bool ServiceClient::isServerConnected(){
    return client_->isServerConnected();
  }

  //****** actionlib::serviceClient *******************
  template <class ActionSpec>
    ServiceClient serviceClient(ros::NodeHandle n, std::string name){
      boost::shared_ptr<ServiceClientImp> client_ptr(new ServiceClientImpT<ActionSpec>(n, name));
      return ServiceClient(client_ptr);
    }

};
#endif
