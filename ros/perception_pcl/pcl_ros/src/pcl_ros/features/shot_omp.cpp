/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Ryohei Ueda, JSK Lab
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
 *   * Neither the name of JSK Lab. nor the names of its
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
 *
 */

#include <pluginlib/class_list_macros.h>
#include "pcl_ros/features/shot_omp.h"

void 
pcl_ros::SHOTEstimationOMP::emptyPublish (const PointCloudInConstPtr &cloud)
{
  PointCloudOut output;
  output.header = cloud->header;
  pub_output_.publish (output.makeShared ());
}

void 
pcl_ros::SHOTEstimationOMP::computePublish (const PointCloudInConstPtr &cloud,
                                            const PointCloudNConstPtr &normals,
                                            const PointCloudInConstPtr &surface,
                                            const IndicesPtr &indices)
{
  // Set the parameters
  impl_.setKSearch (k_);
  impl_.setRadiusSearch (search_radius_);

  // Set the inputs
  impl_.setInputCloud (cloud);
  impl_.setIndices (indices);
  impl_.setSearchSurface (surface);
  impl_.setInputNormals (normals);
  // Estimate the feature
  PointCloudOut output;
  impl_.compute (output);

  // Publish a Boost shared ptr const data
  // Enforce that the TF frame and the timestamp are copied
  output.header = cloud->header;
  pub_output_.publish (output.makeShared ());
}

typedef pcl_ros::SHOTEstimationOMP SHOTEstimationOMP;
PLUGINLIB_DECLARE_CLASS (pcl, SHOTEstimationOMP, SHOTEstimationOMP, nodelet::Nodelet);

