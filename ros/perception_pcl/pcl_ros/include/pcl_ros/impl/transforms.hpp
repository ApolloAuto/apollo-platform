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
 *
 */

#ifndef pcl_ros_IMPL_TRANSFORMS_H_
#define pcl_ros_IMPL_TRANSFORMS_H_

#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

using pcl_conversions::fromPCL;
using pcl_conversions::toPCL;

namespace pcl_ros
{
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
transformPointCloudWithNormals (const pcl::PointCloud <PointT> &cloud_in,
                                pcl::PointCloud <PointT> &cloud_out, const tf::Transform &transform)
{
  // Bullet (used by tf) and Eigen both store quaternions in x,y,z,w order, despite the ordering
  // of arguments in Eigen's constructor. We could use an Eigen Map to convert without copy, but
  // this only works if Bullet uses floats, that is if BT_USE_DOUBLE_PRECISION is not defined.
  // Rather that risking a mistake, we copy the quaternion, which is a small cost compared to
  // the conversion of the point cloud anyway. Idem for the origin.
  tf::Quaternion q = transform.getRotation ();
  Eigen::Quaternionf rotation (q.w (), q.x (), q.y (), q.z ());       // internally stored as (x,y,z,w)
  tf::Vector3 v = transform.getOrigin ();
  Eigen::Vector3f origin (v.x (), v.y (), v.z ());
  //    Eigen::Translation3f translation(v);
  // Assemble an Eigen Transform
  //Eigen::Transform3f t;
  //t = translation * rotation;
  transformPointCloudWithNormals (cloud_in, cloud_out, origin, rotation);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
transformPointCloud (const pcl::PointCloud <PointT> &cloud_in,
                     pcl::PointCloud <PointT> &cloud_out, const tf::Transform &transform)
{
  // Bullet (used by tf) and Eigen both store quaternions in x,y,z,w order, despite the ordering
  // of arguments in Eigen's constructor. We could use an Eigen Map to convert without copy, but
  // this only works if Bullet uses floats, that is if BT_USE_DOUBLE_PRECISION is not defined.
  // Rather that risking a mistake, we copy the quaternion, which is a small cost compared to
  // the conversion of the point cloud anyway. Idem for the origin.
  tf::Quaternion q = transform.getRotation ();
  Eigen::Quaternionf rotation (q.w (), q.x (), q.y (), q.z ());       // internally stored as (x,y,z,w)
  tf::Vector3 v = transform.getOrigin ();
  Eigen::Vector3f origin (v.x (), v.y (), v.z ());
  //    Eigen::Translation3f translation(v);
  // Assemble an Eigen Transform
  //Eigen::Transform3f t;
  //t = translation * rotation;
  transformPointCloud (cloud_in, cloud_out, origin, rotation);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
transformPointCloudWithNormals (const std::string &target_frame,
                                const pcl::PointCloud <PointT> &cloud_in,
                                pcl::PointCloud <PointT> &cloud_out, 
                                const tf::TransformListener &tf_listener)
{
  if (cloud_in.header.frame_id == target_frame)
  {
    cloud_out = cloud_in;
    return (true);
  }

  tf::StampedTransform transform;
  try
  {
    tf_listener.lookupTransform (target_frame, cloud_in.header.frame_id, fromPCL(cloud_in.header).stamp, transform);
  }
  catch (tf::LookupException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }
  catch (tf::ExtrapolationException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }

  transformPointCloudWithNormals (cloud_in, cloud_out, transform);
  cloud_out.header.frame_id = target_frame;
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
transformPointCloud (const std::string &target_frame,
                     const pcl::PointCloud <PointT> &cloud_in,
                     pcl::PointCloud <PointT> &cloud_out, 
                     const tf::TransformListener &tf_listener)
{
  if (cloud_in.header.frame_id == target_frame)
  {
    cloud_out = cloud_in;
    return (true);
  }

  tf::StampedTransform transform;
  try
  {
    tf_listener.lookupTransform (target_frame, cloud_in.header.frame_id, fromPCL(cloud_in.header).stamp, transform);
  }
  catch (tf::LookupException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }
  catch (tf::ExtrapolationException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }
  transformPointCloud (cloud_in, cloud_out, transform);
  cloud_out.header.frame_id = target_frame;
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
transformPointCloud (const std::string &target_frame,
                     const ros::Time & target_time,
                     const pcl::PointCloud <PointT> &cloud_in,
                     const std::string &fixed_frame,
                     pcl::PointCloud <PointT> &cloud_out, 
                     const tf::TransformListener &tf_listener)
{
  tf::StampedTransform transform;
  try
  {
    tf_listener.lookupTransform (target_frame, target_time, cloud_in.header.frame_id, fromPCL(cloud_in.header).stamp, fixed_frame, transform);
  }
  catch (tf::LookupException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }
  catch (tf::ExtrapolationException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }

  transformPointCloud (cloud_in, cloud_out, transform);
  cloud_out.header.frame_id = target_frame;
  std_msgs::Header header;
  header.stamp = target_time;
  cloud_out.header = toPCL(header);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
transformPointCloudWithNormals (const std::string &target_frame,
                                const ros::Time & target_time,
                                const pcl::PointCloud <PointT> &cloud_in,
                                const std::string &fixed_frame,
                                pcl::PointCloud <PointT> &cloud_out, 
                                const tf::TransformListener &tf_listener)
{
  tf::StampedTransform transform;
  try
  {
    tf_listener.lookupTransform (target_frame, target_time, cloud_in.header.frame_id, fromPCL(cloud_in.header).stamp, fixed_frame, transform);
  }
  catch (tf::LookupException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }
  catch (tf::ExtrapolationException &e)
  {
    ROS_ERROR ("%s", e.what ());
    return (false);
  }

  transformPointCloudWithNormals (cloud_in, cloud_out, transform);
  cloud_out.header.frame_id = target_frame;
  std_msgs::Header header;
  header.stamp = target_time;
  cloud_out.header = toPCL(header);
  return (true);
}

}  // namespace pcl_ros
#endif
