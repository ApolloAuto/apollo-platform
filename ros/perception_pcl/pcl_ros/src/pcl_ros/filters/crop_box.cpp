/*
 * 
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
 * $Id: cropbox.cpp 
 *
 */

#include <pluginlib/class_list_macros.h>
#include "pcl_ros/filters/crop_box.h"

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_ros::CropBox::child_init (ros::NodeHandle &nh, bool &has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared <dynamic_reconfigure::Server<pcl_ros::CropBoxConfig> > (nh);
  dynamic_reconfigure::Server<pcl_ros::CropBoxConfig>::CallbackType f = boost::bind (&CropBox::config_callback, this, _1, _2);
  srv_->setCallback (f);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::CropBox::config_callback (pcl_ros::CropBoxConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock lock (mutex_);

  Eigen::Vector4f min_point,max_point; 
  min_point = impl_.getMin();
  max_point = impl_.getMax();

  Eigen::Vector4f new_min_point, new_max_point;
  new_min_point << config.min_x, config.min_y, config.min_z, 0.0;
  new_max_point << config.max_x, config.max_y, config.max_z, 0.0;

  // Check the current values for minimum point
  if (min_point != new_min_point)
  {
    NODELET_DEBUG ("[%s::config_callback] Setting the minimum point to: %f %f %f.", getName ().c_str (), new_min_point(0),new_min_point(1),new_min_point(2));
    // Set the filter min point if different
    impl_.setMin(new_min_point);
  }
 // Check the current values for the maximum point
 if (max_point != new_max_point)
  {
    NODELET_DEBUG ("[%s::config_callback] Setting the maximum point to: %f %f %f.", getName ().c_str (), new_max_point(0),new_max_point(1),new_max_point(2));
    // Set the filter max point if different
    impl_.setMax(new_max_point);
  }

  // Check the current value for keep_organized
  if (impl_.getKeepOrganized () != config.keep_organized)
  {
    NODELET_DEBUG ("[%s::config_callback] Setting the filter keep_organized value to: %s.", getName ().c_str (), config.keep_organized ? "true" : "false");
    // Call the virtual method in the child
    impl_.setKeepOrganized (config.keep_organized);
  }

  // Check the current value for the negative flag
  if (impl_.getNegative() != config.negative)
  {
    NODELET_DEBUG ("[%s::config_callback] Setting the filter negative flag to: %s.", getName ().c_str (), config.negative ? "true" : "false");
    // Call the virtual method in the child
    impl_.setNegative(config.negative);
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

typedef pcl_ros::CropBox CropBox;
PLUGINLIB_EXPORT_CLASS(CropBox,nodelet::Nodelet);

