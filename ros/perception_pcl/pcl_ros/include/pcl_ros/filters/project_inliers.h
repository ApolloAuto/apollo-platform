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
 * $Id: project_inliers.h 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#ifndef PCL_ROS_FILTERS_PROJECT_INLIERS_H_
#define PCL_ROS_FILTERS_PROJECT_INLIERS_H_

// PCL includes
#include <pcl/filters/project_inliers.h>
#include "pcl_ros/filters/filter.h"

#include <message_filters/subscriber.h>

namespace pcl_ros
{
  namespace sync_policies = message_filters::sync_policies;

  /** \brief @b ProjectInliers uses a model and a set of inlier indices from a PointCloud to project them into a
    * separate PointCloud.
    * \note setFilterFieldName (), setFilterLimits (), and setFilterLimitNegative () are ignored.
    * \author Radu Bogdan Rusu
    */
  class ProjectInliers : public Filter
  {
    public:
      ProjectInliers () : model_ () {}

    protected:
      /** \brief Call the actual filter. 
        * \param input the input point cloud dataset
        * \param indices the input set of indices to use from \a input
        * \param output the resultant filtered dataset
        */
      inline void
      filter (const PointCloud2::ConstPtr &input, const IndicesPtr &indices, 
              PointCloud2 &output)
      {
        pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL (*(input), *(pcl_input));
        impl_.setInputCloud (pcl_input);
        impl_.setIndices (indices);
        pcl::ModelCoefficients::Ptr pcl_model(new pcl::ModelCoefficients);
        pcl_conversions::toPCL(*(model_), *(pcl_model));
        impl_.setModelCoefficients (pcl_model);
        pcl::PCLPointCloud2 pcl_output;
        impl_.filter (pcl_output);
        pcl_conversions::moveFromPCL(pcl_output, output);
      }

    private:
      /** \brief A pointer to the vector of model coefficients. */
      ModelCoefficientsConstPtr model_;

      /** \brief The message filter subscriber for model coefficients. */
      message_filters::Subscriber<ModelCoefficients> sub_model_;

      /** \brief Synchronized input, indices, and model coefficients.*/
      boost::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2, PointIndices, ModelCoefficients> > > sync_input_indices_model_e_;
      boost::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2, PointIndices, ModelCoefficients> > > sync_input_indices_model_a_;
      /** \brief The PCL filter implementation used. */
      pcl::ProjectInliers<pcl::PCLPointCloud2> impl_;

      /** \brief Nodelet initialization routine. */
      virtual void 
      onInit ();

      /** \brief PointCloud2 + Indices + Model data callback. */
      void 
      input_indices_model_callback (const PointCloud2::ConstPtr &cloud, 
                                    const PointIndicesConstPtr &indices, 
                                    const ModelCoefficientsConstPtr &model);
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef PCL_FILTERS_PROJECT_INLIERS_H_
