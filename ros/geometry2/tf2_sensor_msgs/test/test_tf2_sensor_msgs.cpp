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


#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>

tf2_ros::Buffer* tf_buffer;
static const double EPS = 1e-3;


TEST(Tf2Sensor, PointCloud2)
{
  sensor_msgs::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(1);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  *iter_x = 1;
  *iter_y = 2;
  *iter_z = 3;

  cloud.header.stamp = ros::Time(2);
  cloud.header.frame_id = "A";

  // simple api
  sensor_msgs::PointCloud2 cloud_simple = tf_buffer->transform(cloud, "B", ros::Duration(2.0));
  sensor_msgs::PointCloud2Iterator<float> iter_x_after(cloud_simple, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_after(cloud_simple, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_after(cloud_simple, "z");
  EXPECT_NEAR(*iter_x_after, -9, EPS);
  EXPECT_NEAR(*iter_y_after, 18, EPS);
  EXPECT_NEAR(*iter_z_after, 27, EPS);

  // advanced api
  sensor_msgs::PointCloud2 cloud_advanced = tf_buffer->transform(cloud, "B", ros::Time(2.0),
                                                                 "A", ros::Duration(3.0));
  sensor_msgs::PointCloud2Iterator<float> iter_x_advanced(cloud_advanced, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_advanced(cloud_advanced, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_advanced(cloud_advanced, "z");
  EXPECT_NEAR(*iter_x_advanced, -9, EPS);
  EXPECT_NEAR(*iter_y_advanced, 18, EPS);
  EXPECT_NEAR(*iter_z_advanced, 27, EPS);
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
  t.transform.rotation.y = 0;
  t.transform.rotation.z = 0;
  t.transform.rotation.w = 0;
  t.header.stamp = ros::Time(2.0);
  t.header.frame_id = "A";
  t.child_frame_id = "B";
  tf_buffer->setTransform(t, "test");

  bool ret = RUN_ALL_TESTS();
  delete tf_buffer;
  return ret;
}
