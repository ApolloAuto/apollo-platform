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

#ifndef TF2_GEOMETRY_MSGS_H
#define TF2_GEOMETRY_MSGS_H

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl/frames.hpp>

namespace tf2
{

inline
KDL::Frame gmTransformToKDL(const geometry_msgs::TransformStamped& t)
  {
    return KDL::Frame(KDL::Rotation::Quaternion(t.transform.rotation.x, t.transform.rotation.y, 
						t.transform.rotation.z, t.transform.rotation.w),
		      KDL::Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
  }


/********************/
/** Vector3Stamped **/
/********************/

// method to extract timestamp from object
template <>
inline
  const ros::Time& getTimestamp(const geometry_msgs::Vector3Stamped& t) {return t.header.stamp;}

// method to extract frame id from object
template <>
inline
  const std::string& getFrameId(const geometry_msgs::Vector3Stamped& t) {return t.header.frame_id;}

// this method needs to be implemented by client library developers
template <>
inline
  void doTransform(const geometry_msgs::Vector3Stamped& t_in, geometry_msgs::Vector3Stamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    KDL::Vector v_out = gmTransformToKDL(transform).M * KDL::Vector(t_in.vector.x, t_in.vector.y, t_in.vector.z);
    t_out.vector.x = v_out[0];
    t_out.vector.y = v_out[1];
    t_out.vector.z = v_out[2];
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }
inline
geometry_msgs::Vector3Stamped toMsg(const geometry_msgs::Vector3Stamped& in)
{
  return in;
}
inline
void fromMsg(const geometry_msgs::Vector3Stamped& msg, geometry_msgs::Vector3Stamped& out)
{
  out = msg;
}



/******************/
/** PointStamped **/
/******************/

// method to extract timestamp from object
template <>
inline
  const ros::Time& getTimestamp(const geometry_msgs::PointStamped& t)  {return t.header.stamp;}

// method to extract frame id from object
template <>
inline
  const std::string& getFrameId(const geometry_msgs::PointStamped& t)  {return t.header.frame_id;}

// this method needs to be implemented by client library developers
template <>
inline
  void doTransform(const geometry_msgs::PointStamped& t_in, geometry_msgs::PointStamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    KDL::Vector v_out = gmTransformToKDL(transform) * KDL::Vector(t_in.point.x, t_in.point.y, t_in.point.z);
    t_out.point.x = v_out[0];
    t_out.point.y = v_out[1];
    t_out.point.z = v_out[2];
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }
inline
geometry_msgs::PointStamped toMsg(const geometry_msgs::PointStamped& in)
{
  return in;
}
inline
void fromMsg(const geometry_msgs::PointStamped& msg, geometry_msgs::PointStamped& out)
{
  out = msg;
}


/*****************/
/** PoseStamped **/
/*****************/

// method to extract timestamp from object
template <>
inline
  const ros::Time& getTimestamp(const geometry_msgs::PoseStamped& t)  {return t.header.stamp;}

// method to extract frame id from object
template <>
inline
  const std::string& getFrameId(const geometry_msgs::PoseStamped& t)  {return t.header.frame_id;}

// this method needs to be implemented by client library developers
template <>
inline
  void doTransform(const geometry_msgs::PoseStamped& t_in, geometry_msgs::PoseStamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    KDL::Vector v(t_in.pose.position.x, t_in.pose.position.y, t_in.pose.position.z);
    KDL::Rotation r = KDL::Rotation::Quaternion(t_in.pose.orientation.x, t_in.pose.orientation.y, t_in.pose.orientation.z, t_in.pose.orientation.w);

    KDL::Frame v_out = gmTransformToKDL(transform) * KDL::Frame(r, v);
    t_out.pose.position.x = v_out.p[0];
    t_out.pose.position.y = v_out.p[1];
    t_out.pose.position.z = v_out.p[2];
    v_out.M.GetQuaternion(t_out.pose.orientation.x, t_out.pose.orientation.y, t_out.pose.orientation.z, t_out.pose.orientation.w);
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }
inline
geometry_msgs::PoseStamped toMsg(const geometry_msgs::PoseStamped& in)
{
  return in;
}
inline
void fromMsg(const geometry_msgs::PoseStamped& msg, geometry_msgs::PoseStamped& out)
{
  out = msg;
}


/****************/
/** Quaternion **/
/****************/

inline
geometry_msgs::Quaternion toMsg(const tf2::Quaternion& in)
{
  geometry_msgs::Quaternion out;
  out.w = in.getW();
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

inline
void fromMsg(const geometry_msgs::Quaternion& in, tf2::Quaternion& out)
{
  // w at the end in the constructor
  out = tf2::Quaternion(in.x, in.y, in.z, in.w);
}


/***********************/
/** QuaternionStamped **/
/***********************/

// method to extract timestamp from object
template <>
inline
const ros::Time& getTimestamp(const geometry_msgs::QuaternionStamped& t)  {return t.header.stamp;}

// method to extract frame id from object
template <>
inline
const std::string& getFrameId(const geometry_msgs::QuaternionStamped& t)  {return t.header.frame_id;}

// this method needs to be implemented by client library developers
template <>
inline
void doTransform(const geometry_msgs::QuaternionStamped& t_in, geometry_msgs::QuaternionStamped& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Quaternion q_out = tf2::Quaternion(transform.transform.rotation.x, transform.transform.rotation.y,
                                          transform.transform.rotation.z, transform.transform.rotation.w)*
                          tf2::Quaternion(t_in.quaternion.x, t_in.quaternion.y, t_in.quaternion.z, t_in.quaternion.w);
  t_out.quaternion = toMsg(q_out);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}
inline
geometry_msgs::QuaternionStamped toMsg(const geometry_msgs::QuaternionStamped& in)
{
  return in;
}
inline
void fromMsg(const geometry_msgs::QuaternionStamped& msg, geometry_msgs::QuaternionStamped& out)
{
  out = msg;
}

template <>
inline
geometry_msgs::QuaternionStamped toMsg(const tf2::Stamped<tf2::Quaternion>& in)
{
  geometry_msgs::QuaternionStamped out;
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  out.quaternion.w = in.getW();
  out.quaternion.x = in.getX();
  out.quaternion.y = in.getY();
  out.quaternion.z = in.getZ();
  return out;
}

template <>
inline
void fromMsg(const geometry_msgs::QuaternionStamped& in, tf2::Stamped<tf2::Quaternion>& out)
{
  out.stamp_ = in.header.stamp;
  out.frame_id_ = in.header.frame_id;
  tf2::Quaternion tmp;
  fromMsg(in.quaternion, tmp);
  out.setData(tmp);
}


/**********************/
/** TransformStamped **/
/**********************/

// method to extract timestamp from object
template <>
inline
const ros::Time& getTimestamp(const geometry_msgs::TransformStamped& t)  {return t.header.stamp;}

// method to extract frame id from object
template <>
inline
const std::string& getFrameId(const geometry_msgs::TransformStamped& t)  {return t.header.frame_id;}

// this method needs to be implemented by client library developers
template <>
inline
void doTransform(const geometry_msgs::TransformStamped& t_in, geometry_msgs::TransformStamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    KDL::Vector v(t_in.transform.translation.x, t_in.transform.translation.y,
                  t_in.transform.translation.z);
    KDL::Rotation r = KDL::Rotation::Quaternion(t_in.transform.rotation.x,
                                                t_in.transform.rotation.y, t_in.transform.rotation.z, t_in.transform.rotation.w);

    KDL::Frame v_out = gmTransformToKDL(transform) * KDL::Frame(r, v);
    t_out.transform.translation.x = v_out.p[0];
    t_out.transform.translation.y = v_out.p[1];
    t_out.transform.translation.z = v_out.p[2];
    v_out.M.GetQuaternion(t_out.transform.rotation.x, t_out.transform.rotation.y,
                          t_out.transform.rotation.z, t_out.transform.rotation.w);
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }
inline
geometry_msgs::TransformStamped toMsg(const geometry_msgs::TransformStamped& in)
{
  return in;
}
inline
void fromMsg(const geometry_msgs::TransformStamped& msg, geometry_msgs::TransformStamped& out)
{
  out = msg;
}


/***************/
/** Transform **/
/***************/

inline
geometry_msgs::Transform toMsg(const tf2::Transform& in)
{
  geometry_msgs::Transform out;
  out.translation.x = in.getOrigin().getX();
  out.translation.y = in.getOrigin().getY();
  out.translation.z = in.getOrigin().getZ();
  out.rotation = toMsg(in.getRotation());
  return out;
}

inline
void fromMsg(const geometry_msgs::Transform& in, tf2::Transform& out)
{
  out.setOrigin(tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
  // w at the end in the constructor
  out.setRotation(tf2::Quaternion(in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));
}


/**********/
/** Pose **/
/**********/

inline
/** This section is about converting */
void toMsg(const tf2::Transform& in, geometry_msgs::Pose& out )
{
  out.position.x = in.getOrigin().getX();
  out.position.y = in.getOrigin().getY();
  out.position.z = in.getOrigin().getZ();
  out.orientation = toMsg(in.getRotation());
}

inline
void fromMsg(const geometry_msgs::Pose& in, tf2::Transform& out)
{
  out.setOrigin(tf2::Vector3(in.position.x, in.position.y, in.position.z));
  // w at the end in the constructor
  out.setRotation(tf2::Quaternion(in.orientation.x, in.orientation.y, in.orientation.z, in.orientation.w));
}

} // namespace

#endif // TF2_GEOMETRY_MSGS_H
