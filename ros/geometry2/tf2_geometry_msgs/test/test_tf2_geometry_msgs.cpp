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


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

tf2_ros::Buffer* tf_buffer;
static const double EPS = 1e-3;


TEST(TfGeometry, Frame)
{
  geometry_msgs::PoseStamped v1;
  v1.pose.position.x = 1;
  v1.pose.position.y = 2;
  v1.pose.position.z = 3;
  v1.pose.orientation.x = 1;
  v1.header.stamp = ros::Time(2);
  v1.header.frame_id = "A";

  // simple api
  geometry_msgs::PoseStamped v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.pose.position.x, -9, EPS);
  EXPECT_NEAR(v_simple.pose.position.y, 18, EPS);
  EXPECT_NEAR(v_simple.pose.position.z, 27, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.x, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.y, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.z, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.w, 1.0, EPS);
  

  // advanced api
  geometry_msgs::PoseStamped v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
							      "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.pose.position.x, -9, EPS);
  EXPECT_NEAR(v_advanced.pose.position.y, 18, EPS);
  EXPECT_NEAR(v_advanced.pose.position.z, 27, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.x, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.y, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.z, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.w, 1.0, EPS);
}



TEST(TfGeometry, Vector)
{
  geometry_msgs::Vector3Stamped v1, res;
  v1.vector.x = 1;
  v1.vector.y = 2;
  v1.vector.z = 3;
  v1.header.stamp = ros::Time(2.0);
  v1.header.frame_id = "A";

  // simple api
  geometry_msgs::Vector3Stamped v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.vector.x, 1, EPS);
  EXPECT_NEAR(v_simple.vector.y, -2, EPS);
  EXPECT_NEAR(v_simple.vector.z, -3, EPS);

  // advanced api
  geometry_msgs::Vector3Stamped v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
								 "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.vector.x, 1, EPS);
  EXPECT_NEAR(v_advanced.vector.y, -2, EPS);
  EXPECT_NEAR(v_advanced.vector.z, -3, EPS);
}


TEST(TfGeometry, Point)
{
  geometry_msgs::PointStamped v1, res;
  v1.point.x = 1;
  v1.point.y = 2;
  v1.point.z = 3;
  v1.header.stamp = ros::Time(2.0);
  v1.header.frame_id = "A";

  // simple api
  geometry_msgs::PointStamped v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.point.x, -9, EPS);
  EXPECT_NEAR(v_simple.point.y, 18, EPS);
  EXPECT_NEAR(v_simple.point.z, 27, EPS);

  // advanced api
  geometry_msgs::PointStamped v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
								 "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.point.x, -9, EPS);
  EXPECT_NEAR(v_advanced.point.y, 18, EPS);
  EXPECT_NEAR(v_advanced.point.z, 27, EPS);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test");
  ros::NodeHandle n;

  tf_buffer = new tf2_ros::Buffer();

  // populate buffer
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = 10;
  t.transform.translation.y = 20;
  t.transform.translation.z = 30;
  t.transform.rotation.x = 1;
  t.header.stamp = ros::Time(2.0);
  t.header.frame_id = "A";
  t.child_frame_id = "B";
  tf_buffer->setTransform(t, "test");

  bool ret = RUN_ALL_TESTS();
  delete tf_buffer;
  return ret;
}





