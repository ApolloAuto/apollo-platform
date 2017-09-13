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
 * $Id: pfh.h 35361 2011-01-20 04:34:49Z rusu $
 *
 */

#ifndef PCL_ROS_PFH_H_
#define PCL_ROS_PFH_H_

#include <pcl/features/pfh.h>
#include "pcl_ros/features/feature.h"

namespace pcl_ros
{
  /** \brief @b PFHEstimation estimates the Point Feature Histogram (PFH) descriptor for a given point cloud dataset
    * containing points and normals.
    *
    * @note If you use this code in any academic work, please cite:
    *
    * <ul>
    * <li> R.B. Rusu, N. Blodow, Z.C. Marton, M. Beetz.
    *      Aligning Point Cloud Views using Persistent Feature Histograms.
    *      In Proceedings of the 21st IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS),
    *      Nice, France, September 22-26 2008.
    * </li>
    * <li> R.B. Rusu, Z.C. Marton, N. Blodow, M. Beetz.
    *      Learning Informative Point Classes for the Acquisition of Object Model Maps.
    *      In Proceedings of the 10th International Conference on Control, Automation, Robotics and Vision (ICARCV),
    *      Hanoi, Vietnam, December 17-20 2008.
    * </li>
    * </ul>
    *
    * @note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
    * \a FPFHEstimationOpenMP for examples on parallel implementations of the FPFH (Fast Point Feature Histogram).
    * \author Radu Bogdan Rusu
    */
  class PFHEstimation : public FeatureFromNormals
  {
    private:
      pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> impl_;

      typedef pcl::PointCloud<pcl::PFHSignature125> PointCloudOut;

      /** \brief Child initialization routine. Internal method. */
      inline bool 
      childInit (ros::NodeHandle &nh)
      {
        // Create the output publisher
        pub_output_ = nh.advertise<PointCloudOut> ("output", max_queue_size_);
        return (true);
      }

      /** \brief Publish an empty point cloud of the feature output type. */
      void emptyPublish (const PointCloudInConstPtr &cloud);

      /** \brief Compute the feature and publish it. */
      void computePublish (const PointCloudInConstPtr &cloud,
                           const PointCloudNConstPtr &normals,
                           const PointCloudInConstPtr &surface,
                           const IndicesPtr &indices);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef PCL_ROS_PFH_H_


