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
 * $Id: pointcloud_to_pcd.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

// ROS core
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace std;

/**
\author Radu Bogdan Rusu

@b pointcloud_to_pcd is a simple node that retrieves a ROS point cloud message and saves it to disk into a PCD (Point
Cloud Data) file format.

**/
class PointCloudToPCD
{
  protected:
    ros::NodeHandle nh_;

  private:
    std::string prefix_;
    bool binary_;
    bool compressed_;

  public:
    string cloud_topic_;

    ros::Subscriber sub_;

    ////////////////////////////////////////////////////////////////////////////////
    // Callback
    void
      cloud_cb (const pcl::PCLPointCloud2::ConstPtr& cloud)
    {
      if ((cloud->width * cloud->height) == 0)
        return;

      ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
                (int)cloud->width * cloud->height,
                cloud->header.frame_id.c_str (),
                pcl::getFieldsList (*cloud).c_str ());

      std::stringstream ss;
      ss << prefix_ << cloud->header.stamp << ".pcd";
      ROS_INFO ("Data saved to %s", ss.str ().c_str ());

      pcl::PCDWriter writer;
      if(binary_)
	{
	  if(compressed_)
	    {
	      writer.writeBinaryCompressed (ss.str (), *cloud, Eigen::Vector4f::Zero (),
					    Eigen::Quaternionf::Identity ());
	    }
	  else
	    {
	      writer.writeBinary (ss.str (), *cloud, Eigen::Vector4f::Zero (),
				  Eigen::Quaternionf::Identity ());
	    }
	}
      else
	{
	  writer.writeASCII (ss.str (), *cloud, Eigen::Vector4f::Zero (),
			     Eigen::Quaternionf::Identity (), 8);
	}

    }

    ////////////////////////////////////////////////////////////////////////////////
    PointCloudToPCD () : binary_(false), compressed_(false)
    {
      // Check if a prefix parameter is defined for output file names.
      ros::NodeHandle priv_nh("~");
      if (priv_nh.getParam ("prefix", prefix_))
        {
          ROS_INFO_STREAM ("PCD file prefix is: " << prefix_);
        }
      else if (nh_.getParam ("prefix", prefix_))
        {
          ROS_WARN_STREAM ("Non-private PCD prefix parameter is DEPRECATED: "
                           << prefix_);
        }

      priv_nh.getParam ("binary", binary_);
      priv_nh.getParam ("compressed", compressed_);
      if(binary_)
	{
	  if(compressed_)
	    {
	      ROS_INFO_STREAM ("Saving as binary compressed PCD");
	    }
	  else
	    {
	      ROS_INFO_STREAM ("Saving as binary PCD");
	    }
	}
      else
	{
	  ROS_INFO_STREAM ("Saving as ASCII PCD");
	}

      cloud_topic_ = "input";

      sub_ = nh_.subscribe (cloud_topic_, 1,  &PointCloudToPCD::cloud_cb, this);
      ROS_INFO ("Listening for incoming data on topic %s",
                nh_.resolveName (cloud_topic_).c_str ());
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "pointcloud_to_pcd", ros::init_options::AnonymousName);

  PointCloudToPCD b;
  ros::spin ();

  return (0);
}
/* ]--- */
