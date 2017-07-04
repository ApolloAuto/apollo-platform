/*
 * Copyright (c) 2014, Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sys/time.h>
#include <rosgraph_msgs/Clock.h>

using namespace tf2;

TEST(tf2_ros_transform_listener, time_backwards)
{

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tfl(buffer);
  tf2_ros::TransformBroadcaster tfb;

  ros::NodeHandle nh = ros::NodeHandle();

  ros::Publisher clock = nh.advertise<rosgraph_msgs::Clock>("/clock", 5);

  rosgraph_msgs::Clock c;
  c.clock = ros::Time(100);
  clock.publish(c);

  // basic test
  ASSERT_FALSE(buffer.canTransform("foo", "bar", ros::Time(101, 0)));

  // set the transform
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time(100, 0);
  msg.header.frame_id = "foo";
  msg.child_frame_id = "bar";
  msg.transform.rotation.x = 1.0;
  tfb.sendTransform(msg);
  msg.header.stamp = ros::Time(102, 0);
  tfb.sendTransform(msg);


  // make sure it arrives
  ros::spinOnce();
  sleep(1);

  // verify it's been set
  ASSERT_TRUE(buffer.canTransform("foo", "bar", ros::Time(101, 0)));

  c.clock = ros::Time(90);
  clock.publish(c);

  // make sure it arrives
  ros::spinOnce();
  sleep(1);

  //Send anoterh message to trigger clock test on an unrelated frame
  msg.header.stamp = ros::Time(110, 0);
  msg.header.frame_id = "foo2";
  msg.child_frame_id = "bar2";
  tfb.sendTransform(msg);

  // make sure it arrives
  ros::spinOnce();
  sleep(1);

  //verify the data's been cleared
  ASSERT_FALSE(buffer.canTransform("foo", "bar", ros::Time(101, 0)));

}




int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "transform_listener_backwards_reset");
  return RUN_ALL_TESTS();
}
