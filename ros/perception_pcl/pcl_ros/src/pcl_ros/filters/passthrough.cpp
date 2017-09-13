/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: passthrough.cpp 36194 2011-02-23 07:49:21Z rusu $
 *
 */

#include <pluginlib/class_list_macros.h>
#include "pcl_ros/filters/passthrough.h"

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_ros::PassThrough::child_init (ros::NodeHandle &nh, bool &has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared <dynamic_reconfigure::Server<pcl_ros::FilterConfig> > (nh);
  dynamic_reconfigure::Server<pcl_ros::FilterConfig>::CallbackType f = boost::bind (&PassThrough::config_callback, this, _1, _2);
  srv_->setCallback (f);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::PassThrough::config_callback (pcl_ros::FilterConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock lock (mutex_);

  double filter_min, filter_max;
  impl_.getFilterLimits (filter_min, filter_max);

  // Check the current values for filter min-max
  if (filter_min != config.filter_limit_min)
  {
    filter_min = config.filter_limit_min;
    NODELET_DEBUG ("[%s::config_callback] Setting the minimum filtering value a point will be considered from to: %f.", getName ().c_str (), filter_min);
    // Set the filter min-max if different
    impl_.setFilterLimits (filter_min, filter_max);
  }
  // Check the current values for filter min-max
  if (filter_max != config.filter_limit_max)
  {
    filter_max = config.filter_limit_max;
    NODELET_DEBUG ("[%s::config_callback] Setting the maximum filtering value a point will be considered from to: %f.", getName ().c_str (), filter_max);
    // Set the filter min-max if different
    impl_.setFilterLimits (filter_min, filter_max);
  }

  // Check the current value for the filter field
  //std::string filter_field = impl_.getFilterFieldName ();
  if (impl_.getFilterFieldName () != config.filter_field_name)
  {
    // Set the filter field if different
    impl_.setFilterFieldName (config.filter_field_name);
    NODELET_DEBUG ("[%s::config_callback] Setting the filter field name to: %s.", getName ().c_str (), config.filter_field_name.c_str ());
  }

  // Check the current value for keep_organized
  if (impl_.getKeepOrganized () != config.keep_organized)
  {
    NODELET_DEBUG ("[%s::config_callback] Setting the filter keep_organized value to: %s.", getName ().c_str (), config.keep_organized ? "true" : "false");
    // Call the virtual method in the child
    impl_.setKeepOrganized (config.keep_organized);
  }

  // Check the current value for the negative flag
  if (impl_.getFilterLimitsNegative () != config.filter_limit_negative)
  {
    NODELET_DEBUG ("[%s::config_callback] Setting the filter negative flag to: %s.", getName ().c_str (), config.filter_limit_negative ? "true" : "false");
    // Call the virtual method in the child
    impl_.setFilterLimitsNegative (config.filter_limit_negative);
  }

  // The following parameters are updated automatically for all PCL_ROS Nodelet Filters as they are inexistent in PCL
  if (tf_input_frame_ != config.input_frame)
  {
    tf_input_frame_ = config.input_frame;
    NODELET_DEBUG ("[%s::config_callback] Setting the input TF frame to: %s.", getName ().c_str (), tf_input_frame_.c_str ());
  }
  if (tf_output_frame_ != config.output_frame)
  {
    tf_output_frame_ = config.output_frame;
    NODELET_DEBUG ("[%s::config_callback] Setting the output TF frame to: %s.", getName ().c_str (), tf_output_frame_.c_str ());
  }
}

typedef pcl_ros::PassThrough PassThrough;
PLUGINLIB_EXPORT_CLASS(PassThrough,nodelet::Nodelet);

