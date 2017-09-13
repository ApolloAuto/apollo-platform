/*
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
 * $Id: radius_outlier_removal.cpp 33319 2010-10-15 04:49:28Z rusu $
 *
 */

#include <pluginlib/class_list_macros.h>
#include "pcl_ros/filters/radius_outlier_removal.h"

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_ros::RadiusOutlierRemoval::child_init (ros::NodeHandle &nh, bool &has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared <dynamic_reconfigure::Server<pcl_ros::RadiusOutlierRemovalConfig> > (nh);
  dynamic_reconfigure::Server<pcl_ros::RadiusOutlierRemovalConfig>::CallbackType f = boost::bind (&RadiusOutlierRemoval::config_callback, this, _1, _2);
  srv_->setCallback (f);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::RadiusOutlierRemoval::config_callback (pcl_ros::RadiusOutlierRemovalConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock lock (mutex_);

  if (impl_.getMinNeighborsInRadius () != config.min_neighbors)
  {
    impl_.setMinNeighborsInRadius (config.min_neighbors);
    NODELET_DEBUG ("[%s::config_callback] Setting the number of neighbors in radius: %d.", getName ().c_str (), config.min_neighbors);
  }

  if (impl_.getRadiusSearch () != config.radius_search)
  {
    impl_.setRadiusSearch (config.radius_search);
    NODELET_DEBUG ("[%s::config_callback] Setting the radius to search neighbors: %f.", getName ().c_str (), config.radius_search);
  }
  
}


typedef pcl_ros::RadiusOutlierRemoval RadiusOutlierRemoval;
PLUGINLIB_EXPORT_CLASS(RadiusOutlierRemoval,nodelet::Nodelet);

