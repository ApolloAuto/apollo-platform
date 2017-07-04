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
#ifndef ACTIONLIB_SERVER_SERVICE_SERVER_IMP_H_
#define ACTIONLIB_SERVER_SERVICE_SERVER_IMP_H_
namespace actionlib {
  template <class ActionSpec>
  ServiceServer advertiseService(ros::NodeHandle n, std::string name,
          boost::function<bool (const typename ActionSpec::_action_goal_type::_goal_type&, 
                                typename ActionSpec::_action_result_type::_result_type& result)> service_cb)
  {
    boost::shared_ptr<ServiceServerImp> server_ptr(new ServiceServerImpT<ActionSpec>(n, name, service_cb));
    return ServiceServer(server_ptr);
  }

  template <class ActionSpec>
  ServiceServerImpT<ActionSpec>::ServiceServerImpT(ros::NodeHandle n, std::string name, 
      boost::function<bool (const Goal&, Result& result)> service_cb)
      : service_cb_(service_cb)
  {
    as_ = boost::shared_ptr<ActionServer<ActionSpec> >(new ActionServer<ActionSpec>(n, name,
          boost::bind(&ServiceServerImpT::goalCB, this, _1), false));
    as_->start();
  }

  template <class ActionSpec>
  void ServiceServerImpT<ActionSpec>::goalCB(GoalHandle goal){
    goal.setAccepted("This goal has been accepted by the service server");

    //we need to pass the result into the users callback
    Result r;
    if(service_cb_(*(goal.getGoal()), r))
      goal.setSucceeded(r, "The service server successfully processed the request");
    else
      goal.setAborted(r, "The service server failed to process the request");
  }
};
#endif
