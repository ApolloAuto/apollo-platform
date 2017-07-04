/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Wim Meeussen */

#ifndef TF2_KDL_H
#define TF2_KDL_H

#include <tf2/convert.h>
#include <kdl/frames.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>


namespace tf2
{
inline
KDL::Frame transformToKDL(const geometry_msgs::TransformStamped& t)
  {
    return KDL::Frame(KDL::Rotation::Quaternion(t.transform.rotation.x, t.transform.rotation.y, 
						t.transform.rotation.z, t.transform.rotation.w),
		      KDL::Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
  }

inline
geometry_msgs::TransformStamped kdlToTransform(const KDL::Frame& k)
{
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = k.p.x();
  t.transform.translation.y = k.p.y();
  t.transform.translation.z = k.p.z();
  k.M.GetQuaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
  return t;
}

// ---------------------
// Vector
// ---------------------
// this method needs to be implemented by client library developers
template <>
inline
  void doTransform(const tf2::Stamped<KDL::Vector>& t_in, tf2::Stamped<KDL::Vector>& t_out, const geometry_msgs::TransformStamped& transform)
  {
    t_out = tf2::Stamped<KDL::Vector>(transformToKDL(transform) * t_in, transform.header.stamp, transform.header.frame_id);
  }

//convert to vector message
inline
geometry_msgs::PointStamped toMsg(const tf2::Stamped<KDL::Vector>& in)
{
  geometry_msgs::PointStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.point.x = in[0];
  msg.point.y = in[1];
  msg.point.z = in[2];
  return msg;
}

inline
void fromMsg(const geometry_msgs::PointStamped& msg, tf2::Stamped<KDL::Vector>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  out[0] = msg.point.x;
  out[1] = msg.point.y;
  out[2] = msg.point.z;
}

// ---------------------
// Twist
// ---------------------
// this method needs to be implemented by client library developers
template <>
inline
  void doTransform(const tf2::Stamped<KDL::Twist>& t_in, tf2::Stamped<KDL::Twist>& t_out, const geometry_msgs::TransformStamped& transform)
  {
    t_out = tf2::Stamped<KDL::Twist>(transformToKDL(transform) * t_in, transform.header.stamp, transform.header.frame_id);
  }

//convert to twist message
inline
geometry_msgs::TwistStamped toMsg(const tf2::Stamped<KDL::Twist>& in)
{
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.twist.linear.x = in.vel[0];
  msg.twist.linear.y = in.vel[1];
  msg.twist.linear.z = in.vel[2];
  msg.twist.angular.x = in.rot[0];
  msg.twist.angular.y = in.rot[1];
  msg.twist.angular.z = in.rot[2];
  return msg;
}

inline
void fromMsg(const geometry_msgs::TwistStamped& msg, tf2::Stamped<KDL::Twist>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  out.vel[0] = msg.twist.linear.x;
  out.vel[1] = msg.twist.linear.y;
  out.vel[2] = msg.twist.linear.z;
  out.rot[0] = msg.twist.angular.x;
  out.rot[1] = msg.twist.angular.y;
  out.rot[2] = msg.twist.angular.z;
}


// ---------------------
// Wrench
// ---------------------
// this method needs to be implemented by client library developers
template <>
inline
  void doTransform(const tf2::Stamped<KDL::Wrench>& t_in, tf2::Stamped<KDL::Wrench>& t_out, const geometry_msgs::TransformStamped& transform)
  {
    t_out = tf2::Stamped<KDL::Wrench>(transformToKDL(transform) * t_in, transform.header.stamp, transform.header.frame_id);
  }

//convert to wrench message
inline
geometry_msgs::WrenchStamped toMsg(const tf2::Stamped<KDL::Wrench>& in)
{
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.wrench.force.x = in.force[0];
  msg.wrench.force.y = in.force[1];
  msg.wrench.force.z = in.force[2];
  msg.wrench.torque.x = in.torque[0];
  msg.wrench.torque.y = in.torque[1];
  msg.wrench.torque.z = in.torque[2];
  return msg;
}

inline
void fromMsg(const geometry_msgs::WrenchStamped& msg, tf2::Stamped<KDL::Wrench>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  out.force[0] = msg.wrench.force.x;
  out.force[1] = msg.wrench.force.y;
  out.force[2] = msg.wrench.force.z;
  out.torque[0] = msg.wrench.torque.x;
  out.torque[1] = msg.wrench.torque.y;
  out.torque[2] = msg.wrench.torque.z;
}




// ---------------------
// Frame
// ---------------------
// this method needs to be implemented by client library developers
template <>
inline
  void doTransform(const tf2::Stamped<KDL::Frame>& t_in, tf2::Stamped<KDL::Frame>& t_out, const geometry_msgs::TransformStamped& transform)
  {
    t_out = tf2::Stamped<KDL::Frame>(transformToKDL(transform) * t_in, transform.header.stamp, transform.header.frame_id);
  }

//convert to pose message
inline
geometry_msgs::PoseStamped toMsg(const tf2::Stamped<KDL::Frame>& in)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.pose.position.x = in.p[0];
  msg.pose.position.y = in.p[1];
  msg.pose.position.z = in.p[2];
  in.M.GetQuaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  return msg;
}

inline
void fromMsg(const geometry_msgs::PoseStamped& msg, tf2::Stamped<KDL::Frame>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  out.p[0] = msg.pose.position.x;
  out.p[1] = msg.pose.position.y;
  out.p[2] = msg.pose.position.z;
  out.M = KDL::Rotation::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
}


} // namespace

#endif // TF2_KDL_H
