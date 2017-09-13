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
 * $Id: pcd_to_pointcloud.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

@b pcd_to_pointcloud is a simple node that loads PCD (Point Cloud Data) files from disk and publishes them as ROS messages on the network.

 **/

// ROS core
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl_ros/publisher.h"

using namespace std;

class PCDGenerator
{
  protected:
    string tf_frame_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
  public:

    // ROS messages
    sensor_msgs::PointCloud2 cloud_;

    string file_name_, cloud_topic_;
    double wait_;

    pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_;

    ////////////////////////////////////////////////////////////////////////////////
    PCDGenerator () : tf_frame_ ("/base_link"), private_nh_("~")
    {
      // Maximum number of outgoing messages to be queued for delivery to subscribers = 1

      cloud_topic_ = "cloud_pcd";
      pub_.advertise (nh_, cloud_topic_.c_str (), 1);
      private_nh_.param("frame_id", tf_frame_, std::string("/base_link"));
      ROS_INFO ("Publishing data on topic %s with frame_id %s.", nh_.resolveName (cloud_topic_).c_str (), tf_frame_.c_str());
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Start
    int
      start ()
    {
      if (file_name_ == "" || pcl::io::loadPCDFile (file_name_, cloud_) == -1)
        return (-1);
      cloud_.header.frame_id = tf_frame_;
      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
      int nr_points      = cloud_.width * cloud_.height;
      string fields_list = pcl::getFieldsList (cloud_);
      double interval = wait_ * 1e+6;
      while (nh_.ok ())
      {
        ROS_DEBUG_ONCE ("Publishing data with %d points (%s) on topic %s in frame %s.", nr_points, fields_list.c_str (), nh_.resolveName (cloud_topic_).c_str (), cloud_.header.frame_id.c_str ());
        cloud_.header.stamp = ros::Time::now ();

        if (pub_.getNumSubscribers () > 0)
        {
          ROS_DEBUG ("Publishing data to %d subscribers.", pub_.getNumSubscribers ());
          pub_.publish (cloud_);
        }
        else
        {
					// check once a second if there is any subscriber
          ros::Duration (1).sleep ();
          continue;
        }

        usleep (interval);

        if (interval == 0)	// We only publish once if a 0 seconds interval is given
				{
					// Give subscribers 3 seconds until point cloud decays... a little ugly!
		      ros::Duration (3.0).sleep ();
          break;
				}
      }
      return (true);
    }


};

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Syntax is: " << argv[0] << " <file.pcd> [publishing_interval (in seconds)]" << std::endl;
    return (-1);
  }

  ros::init (argc, argv, "pcd_to_pointcloud");

  PCDGenerator c;
  c.file_name_ = string (argv[1]);
  // check if publishing interval is given
  if (argc == 2)
	{
  	c.wait_ = 0;
	}
	else
	{
		c.wait_ = atof (argv[2]);
	}

  if (c.start () == -1)
  {
    ROS_ERROR ("Could not load file %s. Exiting.", argv[1]);
    return (-1);
  }
  ROS_INFO ("Loaded a point cloud with %d points (total size is %zu) and the following channels: %s.",  c.cloud_.width * c.cloud_.height, c.cloud_.data.size (), pcl::getFieldsList (c.cloud_).c_str ());
  c.spin ();

  return (0);
}
/* ]--- */
