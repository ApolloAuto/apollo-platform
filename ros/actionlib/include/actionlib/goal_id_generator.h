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

#ifndef ACTION_LIB_GOAL_ID_GENERATOR_H_
#define ACTION_LIB_GOAL_ID_GENERATOR_H_

#include <sstream>
#include <string>
#include "ros/time.h"
#include "actionlib_msgs/GoalID.h"
#include <actionlib/decl.h>

namespace actionlib
{

class ACTIONLIB_DECL GoalIDGenerator
{
public:

  /**
   * Create a generator that prepends the fully qualified node name to the Goal ID
   */
  GoalIDGenerator();

  /**
   * \param name Unique name to prepend to the goal id. This will
   *             generally be a fully qualified node name.
   */
  GoalIDGenerator(const std::string& name);

  /**
   * \param name Set the name to prepend to the goal id. This will
   *             generally be a fully qualified node name.
   */
  void setName(const std::string& name);

  /**
   * \brief Generates a unique ID
   * \return A unique GoalID for this action
   */
  actionlib_msgs::GoalID generateID();

private:
  std::string name_ ;

};

}

#endif
