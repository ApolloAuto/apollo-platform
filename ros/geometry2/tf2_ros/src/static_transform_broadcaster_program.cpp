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

#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/static_transform_broadcaster.h"

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv,"static_transform_publisher", ros::init_options::AnonymousName);
  tf2_ros::StaticTransformBroadcaster broadcaster;

  if(argc == 10)
  {

    if (strcmp(argv[8], argv[9]) == 0)
    {
      ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", argv[8], argv[9]);
      return 1;
    }

    geometry_msgs::TransformStamped msg;
    msg.transform.translation.x = atof(argv[1]);
    msg.transform.translation.y = atof(argv[2]);
    msg.transform.translation.z = atof(argv[3]);
    msg.transform.rotation.x = atof(argv[4]);
    msg.transform.rotation.y = atof(argv[5]);
    msg.transform.rotation.z = atof(argv[6]);
    msg.transform.rotation.w = atof(argv[7]);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = argv[8];
    msg.child_frame_id = argv[9];
  


  broadcaster.sendTransform(msg);
  ROS_INFO("Spinning until killed publishing %s to %s", msg.header.frame_id.c_str(), msg.child_frame_id.c_str());
  ros::spin();

  return 0;
} 
  else if (argc == 9)
  {
    if (strcmp(argv[7], argv[8]) == 0)
    {
      ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", argv[8], argv[9]);
      return 1;
    }
    
    geometry_msgs::TransformStamped msg;
    msg.transform.translation.x = atof(argv[1]);
    msg.transform.translation.y = atof(argv[2]);
    msg.transform.translation.z = atof(argv[3]);

    tf2::Quaternion quat;
    quat.setRPY(atof(argv[6]), atof(argv[5]), atof(argv[4]));
    msg.transform.rotation.x = quat.x();
    msg.transform.rotation.y = quat.y();
    msg.transform.rotation.z = quat.z();
    msg.transform.rotation.w = quat.w();

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = argv[7];
    msg.child_frame_id = argv[8];

    broadcaster.sendTransform(msg);
    ROS_INFO("Spinning until killed publishing %s to %s", msg.header.frame_id.c_str(), msg.child_frame_id.c_str());
    ros::spin();
    return 0;
  }
  else
  {
    printf("A command line utility for manually sending a transform.\n");
    //printf("It will periodicaly republish the given transform. \n");
    printf("Usage: static_transform_publisher x y z qx qy qz qw frame_id child_frame_id \n");
    printf("OR \n");
    printf("Usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id \n");
    printf("\nThis transform is the transform of the coordinate frame from frame_id into the coordinate frame \n");
    printf("of the child_frame_id.  \n");
    ROS_ERROR("static_transform_publisher exited due to not having the right number of arguments");
    return -1;
  }


};

