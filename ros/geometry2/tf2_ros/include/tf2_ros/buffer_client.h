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
#ifndef TF2_ROS_BUFFER_CLIENT_H_
#define TF2_ROS_BUFFER_CLIENT_H_

#include <tf2_ros/buffer_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_msgs/LookupTransformAction.h>

namespace tf2_ros
{
  class BufferClient : public BufferInterface
  {
    public:
      typedef actionlib::SimpleActionClient<tf2_msgs::LookupTransformAction> LookupActionClient;

      /** \brief BufferClient constructor
       * \param ns The namespace in which to look for a BufferServer
       * \param check_frequency The frequency in Hz to check whether the BufferServer has completed a request
       * \param timeout_padding The amount of time to allow passed the desired timeout on the client side for communication lag
       */
      BufferClient(std::string ns, double check_frequency = 10.0, ros::Duration timeout_padding = ros::Duration(2.0));

      /** \brief Get the transform between two frames by frame ID.
       * \param target_frame The frame to which data should be transformed
       * \param source_frame The frame where the data originated
       * \param time The time at which the value of the transform is desired. (0 will get the latest)
       * \param timeout How long to block before failing
       * \return The transform between the frames
       *
       * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
       * tf2::ExtrapolationException, tf2::InvalidArgumentException
       */
      virtual geometry_msgs::TransformStamped
        lookupTransform(const std::string& target_frame, const std::string& source_frame,
            const ros::Time& time, const ros::Duration timeout = ros::Duration(0.0)) const;

      /** \brief Get the transform between two frames by frame ID assuming fixed frame.
       * \param target_frame The frame to which data should be transformed
       * \param target_time The time to which the data should be transformed. (0 will get the latest)
       * \param source_frame The frame where the data originated
       * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
       * \param fixed_frame The frame in which to assume the transform is constant in time. 
       * \param timeout How long to block before failing
       * \return The transform between the frames
       *
       * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
       * tf2::ExtrapolationException, tf2::InvalidArgumentException
       */
      virtual geometry_msgs::TransformStamped 
        lookupTransform(const std::string& target_frame, const ros::Time& target_time,
            const std::string& source_frame, const ros::Time& source_time,
            const std::string& fixed_frame, const ros::Duration timeout = ros::Duration(0.0)) const;

      /** \brief Test if a transform is possible
       * \param target_frame The frame into which to transform
       * \param source_frame The frame from which to transform
       * \param time The time at which to transform
       * \param timeout How long to block before failing
       * \param errstr A pointer to a string which will be filled with why the transform failed, if not NULL
       * \return True if the transform is possible, false otherwise 
       */
      virtual bool
        canTransform(const std::string& target_frame, const std::string& source_frame, 
            const ros::Time& time, const ros::Duration timeout = ros::Duration(0.0), std::string* errstr = NULL) const;

      /** \brief Test if a transform is possible
       * \param target_frame The frame into which to transform
       * \param target_time The time into which to transform
       * \param source_frame The frame from which to transform
       * \param source_time The time from which to transform
       * \param fixed_frame The frame in which to treat the transform as constant in time
       * \param timeout How long to block before failing
       * \param errstr A pointer to a string which will be filled with why the transform failed, if not NULL
       * \return True if the transform is possible, false otherwise 
       */
      virtual bool
        canTransform(const std::string& target_frame, const ros::Time& target_time,
            const std::string& source_frame, const ros::Time& source_time,
            const std::string& fixed_frame, const ros::Duration timeout = ros::Duration(0.0), std::string* errstr = NULL) const;

      bool waitForServer(const ros::Duration& timeout = ros::Duration(0))
      {
        return client_.waitForServer(timeout);
      }

    private:
      geometry_msgs::TransformStamped processGoal(const tf2_msgs::LookupTransformGoal& goal) const;
      geometry_msgs::TransformStamped processResult(const tf2_msgs::LookupTransformResult& result) const;
      mutable LookupActionClient client_;
      double check_frequency_;
      ros::Duration timeout_padding_;
  };
};
#endif
