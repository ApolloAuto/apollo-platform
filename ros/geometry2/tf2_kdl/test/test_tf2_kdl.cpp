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


#include <tf2_kdl/tf2_kdl.h>
#include <kdl/frames_io.hpp>
#include <gtest/gtest.h>
#include "tf2_ros/buffer.h"


tf2_ros::Buffer* tf_buffer;
static const double EPS = 1e-3;

TEST(TfKDL, Frame)
{
  tf2::Stamped<KDL::Frame> v1(KDL::Frame(KDL::Rotation::RPY(M_PI, 0, 0), KDL::Vector(1,2,3)), ros::Time(2.0), "A");


  // simple api
  KDL::Frame v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple.p[0], -9, EPS);
  EXPECT_NEAR(v_simple.p[1], 18, EPS);
  EXPECT_NEAR(v_simple.p[2], 27, EPS);
  double r, p, y;
  v_simple.M.GetRPY(r, p, y);
  EXPECT_NEAR(r, 0.0, EPS);
  EXPECT_NEAR(p, 0.0, EPS);
  EXPECT_NEAR(y, 0.0, EPS);
  

  // advanced api
  KDL::Frame v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
					       "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced.p[0], -9, EPS);
  EXPECT_NEAR(v_advanced.p[1], 18, EPS);
  EXPECT_NEAR(v_advanced.p[2], 27, EPS);
  v_advanced.M.GetRPY(r, p, y);
  EXPECT_NEAR(r, 0.0, EPS);
  EXPECT_NEAR(p, 0.0, EPS);
  EXPECT_NEAR(y, 0.0, EPS);

}



TEST(TfKDL, Vector)
{
  tf2::Stamped<KDL::Vector> v1(KDL::Vector(1,2,3), ros::Time(2.0), "A");


  // simple api
  KDL::Vector v_simple = tf_buffer->transform(v1, "B", ros::Duration(2.0));
  EXPECT_NEAR(v_simple[0], -9, EPS);
  EXPECT_NEAR(v_simple[1], 18, EPS);
  EXPECT_NEAR(v_simple[2], 27, EPS);

  // advanced api
  KDL::Vector v_advanced = tf_buffer->transform(v1, "B", ros::Time(2.0),
					       "A", ros::Duration(3.0));
  EXPECT_NEAR(v_advanced[0], -9, EPS);
  EXPECT_NEAR(v_advanced[1], 18, EPS);
  EXPECT_NEAR(v_advanced[2], 27, EPS);
}

TEST(TfKDL, ConvertVector)
{
  tf2::Stamped<KDL::Vector> v(KDL::Vector(1,2,3), ros::Time(), "my_frame");

  tf2::Stamped<KDL::Vector> v1 = v;
  tf2::convert(v1, v1);

  EXPECT_EQ(v, v1);

  tf2::Stamped<KDL::Vector> v2(KDL::Vector(3,4,5), ros::Time(), "my_frame2");
  tf2::convert(v1, v2);

  EXPECT_EQ(v, v2);
  EXPECT_EQ(v1, v2);
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

  bool retval = RUN_ALL_TESTS();
  delete tf_buffer;
  return retval;
}
