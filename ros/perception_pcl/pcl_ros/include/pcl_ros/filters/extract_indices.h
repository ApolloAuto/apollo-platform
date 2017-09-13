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
 * $Id: extract_indices.h 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#ifndef PCL_ROS_FILTERS_EXTRACTINDICES_H_
#define PCL_ROS_FILTERS_EXTRACTINDICES_H_

// PCL includes
#include <pcl/filters/extract_indices.h>

#include "pcl_ros/filters/filter.h"
#include "pcl_ros/ExtractIndicesConfig.h"

namespace pcl_ros
{
  /** \brief @b ExtractIndices extracts a set of indices from a PointCloud as a separate PointCloud.
    * \note setFilterFieldName (), setFilterLimits (), and setFilterLimitNegative () are ignored.
    * \author Radu Bogdan Rusu
    */
  class ExtractIndices : public Filter
  {
    protected:
      /** \brief Pointer to a dynamic reconfigure service. */
      boost::shared_ptr <dynamic_reconfigure::Server<pcl_ros::ExtractIndicesConfig> > srv_;

      /** \brief Call the actual filter. 
        * \param input the input point cloud dataset
        * \param indices the input set of indices to use from \a input
        * \param output the resultant filtered dataset
        */
      inline void
      filter (const PointCloud2::ConstPtr &input, const IndicesPtr &indices, 
              PointCloud2 &output)
      {
        boost::mutex::scoped_lock lock (mutex_);
        pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*(input), *(pcl_input));
        impl_.setInputCloud (pcl_input);
        impl_.setIndices (indices);
        pcl::PCLPointCloud2 pcl_output;
        impl_.filter (pcl_output);
        pcl_conversions::moveFromPCL(pcl_output, output);
      }

      /** \brief Child initialization routine.
        * \param nh ROS node handle
        * \param has_service set to true if the child has a Dynamic Reconfigure service
        */
      virtual bool 
      child_init (ros::NodeHandle &nh, bool &has_service);

      /** \brief Dynamic reconfigure service callback. */
      void 
      config_callback (pcl_ros::ExtractIndicesConfig &config, uint32_t level);

    private:
      /** \brief The PCL filter implementation used. */
      pcl::ExtractIndices<pcl::PCLPointCloud2> impl_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef PCL_ROS_FILTERS_EXTRACTINDICES_H_

