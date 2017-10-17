/* $Id$ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Jack O'Quin
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
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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
*********************************************************************/

#include <unistd.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "camera_info_manager/camera_info_manager.h"
#include <sensor_msgs/distortion_models.h>
#include <string>
#include <gtest/gtest.h>

///////////////////////////////////////////////////////////////
// global test data
///////////////////////////////////////////////////////////////

namespace
{
  const std::string g_package_name("camera_info_manager");
  const std::string g_test_name("test_calibration");
  const std::string g_package_filename("/tests/" + g_test_name +".yaml");
  const std::string g_package_url("package://" + g_package_name
                                  + g_package_filename);
  const std::string g_package_name_url("package://" + g_package_name
                                       + "/tests/${NAME}.yaml");
  const std::string g_default_url("file://${ROS_HOME}/camera_info/${NAME}.yaml");
  const std::string g_camera_name("08144361026320a0");
}

///////////////////////////////////////////////////////////////
// utility functions
///////////////////////////////////////////////////////////////

// compare CameraInfo fields that are saved and loaded for calibration
void compare_calibration(const sensor_msgs::CameraInfo &exp,
                         const sensor_msgs::CameraInfo &ci)
{
  // check image size
  EXPECT_EQ(exp.width, ci.width);
  EXPECT_EQ(exp.height, ci.height);

  // check distortion coefficients
  EXPECT_EQ(exp.distortion_model, ci.distortion_model);
  EXPECT_EQ(exp.D.size(), ci.D.size());
  for (unsigned i = 0; i < ci.D.size(); ++i)
    {
      EXPECT_EQ(exp.D[i], ci.D[i]);
    }

  // check camera matrix
  for (unsigned i = 0; i < ci.K.size(); ++i)
    {
      EXPECT_EQ(exp.K[i], ci.K[i]);
    }

  // check rectification matrix
  for (unsigned i = 0; i < ci.R.size(); ++i)
    {
      EXPECT_EQ(exp.R[i], ci.R[i]);
    }

  // check projection matrix
  for (unsigned i = 0; i < ci.P.size(); ++i)
    {
      EXPECT_EQ(exp.P[i], ci.P[i]);
    }
}

// make sure this file does not exist
void delete_file(std::string filename)
{
  int rc = unlink(filename.c_str());
  if (rc != 0)
    {
      if (errno != ENOENT)
        ROS_INFO_STREAM("unexpected unlink() error: " << errno);
    }
}

void delete_default_file(void)
{
  std::string ros_home("/tmp");
  setenv("ROS_HOME", ros_home.c_str(), true);
  std::string tmpFile(ros_home + "/camera_info/camera.yaml");
  delete_file(tmpFile);
}

void do_system(const std::string &command)
{
  int rc = system(command.c_str());
  if (rc)
    std::cout << command << " returns " << rc;
}

void delete_tmp_camera_info_directory(void)
{
  do_system(std::string("rm -rf /tmp/camera_info"));
}

void make_tmp_camera_info_directory(void)
{
  do_system(std::string("mkdir -p /tmp/camera_info"));
}

// These data must match the contents of test_calibration.yaml.
sensor_msgs::CameraInfo expected_calibration(void)
{
  sensor_msgs::CameraInfo ci;

  ci.width = 640u;
  ci.height = 480u;

  // set distortion coefficients
  ci.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  ci.D.resize(5);
  ci.D[0] = -1.04482;
  ci.D[1] = 1.59252;
  ci.D[2] = -0.0196308;
  ci.D[3] = 0.0287906;
  ci.D[4] = 0.0;

  // set camera matrix
  ci.K[0] = 1168.68;
  ci.K[1] = 0.0;
  ci.K[2] = 295.015;
  ci.K[3] = 0.0;
  ci.K[4] = 1169.01;
  ci.K[5] = 252.247;
  ci.K[6] = 0.0;
  ci.K[7] = 0.0;
  ci.K[8] = 1.0;

  // set rectification matrix
  ci.R[0] = 1.0;
  ci.R[1] = 0.0;
  ci.R[2] = 0.0;
  ci.R[3] = 0.0;
  ci.R[4] = 1.0;
  ci.R[5] = 0.0;
  ci.R[6] = 0.0;
  ci.R[7] = 0.0;
  ci.R[8] = 1.0;

  // set projection matrix
  ci.P[0] = ci.K[0];
  ci.P[1] = ci.K[1];
  ci.P[2] = ci.K[2];
  ci.P[3] = 0.0;
  ci.P[4] = ci.K[3];
  ci.P[5] = ci.K[4];
  ci.P[6] = ci.K[5];
  ci.P[7] = 0.0;
  ci.P[8] = ci.K[6];
  ci.P[9] = ci.K[7];
  ci.P[10] = ci.K[8];
  ci.P[11] = 0.0;

  return ci;
}

// issue SetCameraInfo service request
bool set_calibration(ros::NodeHandle node,
                     const sensor_msgs::CameraInfo &calib)
{
  ros::ServiceClient client =
    node.serviceClient<sensor_msgs::SetCameraInfo>("set_camera_info");
  sensor_msgs::SetCameraInfo set_camera_info;
  set_camera_info.request.camera_info = calib;
  bool success;
  EXPECT_TRUE((success = client.call(set_camera_info)));
  return success;
}

// resolve URL string, result should be as expected
void check_url_substitution(ros::NodeHandle node,
                            const std::string &url,
                            const std::string &exp_url,
                            const std::string &camera_name)
{
  camera_info_manager::CameraInfoManager cinfo(node, camera_name, url);
  std::string sub_url = cinfo.resolveURL(url, camera_name);
  EXPECT_EQ(sub_url, exp_url);
}

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

// Test that valid camera names are accepted
TEST(CameraName, validNames)
{
  ros::NodeHandle node;
  camera_info_manager::CameraInfoManager cinfo(node);

  EXPECT_TRUE(cinfo.setCameraName(std::string("a")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("1")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("_")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("abcdefghijklmnopqrstuvwxyz")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("ABCDEFGHIJKLMNOPQRSTUVWXYZ")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("0123456789")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("0123456789abcdef")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("A1")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("9z")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("08144361026320a0_640x480_mono8")));

}

// Test that invalid camera names are rejected
TEST(CameraName, invalidNames)
{
  ros::NodeHandle node;
  camera_info_manager::CameraInfoManager cinfo(node);

  EXPECT_FALSE(cinfo.setCameraName(std::string("")));
  EXPECT_FALSE(cinfo.setCameraName(std::string("-21")));
  EXPECT_FALSE(cinfo.setCameraName(std::string("C++")));
  EXPECT_FALSE(cinfo.setCameraName(std::string("file:///tmp/url.yaml")));
  EXPECT_FALSE(cinfo.setCameraName(std::string("file://${INVALID}/xxx.yaml")));
}

// Test that valid URLs are accepted
TEST(UrlValidation, validURLs)
{
  ros::NodeHandle node;
  camera_info_manager::CameraInfoManager cinfo(node);

  EXPECT_TRUE(cinfo.validateURL(std::string("")));
  EXPECT_TRUE(cinfo.validateURL(std::string("file:///")));
  EXPECT_TRUE(cinfo.validateURL(std::string("file:///tmp/url.yaml")));
  EXPECT_TRUE(cinfo.validateURL(std::string("File:///tmp/url.ini")));
  EXPECT_TRUE(cinfo.validateURL(std::string("FILE:///tmp/url.yaml")));
  EXPECT_TRUE(cinfo.validateURL(g_default_url));
  EXPECT_TRUE(cinfo.validateURL(g_package_url));
  EXPECT_TRUE(cinfo.validateURL(std::string("package://no_such_package/calibration.yaml")));
  EXPECT_TRUE(cinfo.validateURL(std::string("packAge://camera_info_manager/x")));
}

// Test that invalid URLs are rejected
TEST(UrlValidation, invalidURLs)
{
  ros::NodeHandle node;
  camera_info_manager::CameraInfoManager cinfo(node);

  EXPECT_FALSE(cinfo.validateURL(std::string("file://")));
  EXPECT_FALSE(cinfo.validateURL(std::string("flash:///")));
  EXPECT_FALSE(cinfo.validateURL(std::string("html://ros.org/wiki/camera_info_manager")));
  EXPECT_FALSE(cinfo.validateURL(std::string("package://")));
  EXPECT_FALSE(cinfo.validateURL(std::string("package:///")));
  EXPECT_FALSE(cinfo.validateURL(std::string("package://calibration.yaml")));
  EXPECT_FALSE(cinfo.validateURL(std::string("package://camera_info_manager/")));
}

// Test ability to provide uncalibrated CameraInfo
TEST(GetInfo, uncalibrated)
{
  ros::NodeHandle node;

  delete_default_file();

  camera_info_manager::CameraInfoManager cinfo(node);
  EXPECT_FALSE(cinfo.isCalibrated());

  sensor_msgs::CameraInfo ci(cinfo.getCameraInfo());
  sensor_msgs::CameraInfo exp;
  compare_calibration(exp, ci);
}

// Test ability to load calibrated CameraInfo
TEST(GetInfo, calibrated)
{
  ros::NodeHandle node;

  delete_default_file();

  camera_info_manager::CameraInfoManager cinfo(node);
  EXPECT_FALSE(cinfo.isCalibrated());

  std::string pkgPath(ros::package::getPath(g_package_name));
  std::string url("file://" + pkgPath + "/tests/test_calibration.yaml");
  EXPECT_TRUE(cinfo.loadCameraInfo(url));
  EXPECT_TRUE(cinfo.isCalibrated());

  sensor_msgs::CameraInfo ci(cinfo.getCameraInfo());
  sensor_msgs::CameraInfo exp(expected_calibration());
  compare_calibration(exp, ci);
}

// Test ability to load calibrated CameraInfo from package
TEST(GetInfo, fromPackage)
{
  ros::NodeHandle node;
  camera_info_manager::CameraInfoManager cinfo(node);

  EXPECT_TRUE(cinfo.loadCameraInfo(g_package_url));
  EXPECT_TRUE(cinfo.isCalibrated());

  sensor_msgs::CameraInfo ci(cinfo.getCameraInfo());
  sensor_msgs::CameraInfo exp(expected_calibration());
  compare_calibration(exp, ci);
}

// Test ability to access named calibrated CameraInfo from package
TEST(GetInfo, fromPackageWithName)
{
  ros::NodeHandle node;
  camera_info_manager::CameraInfoManager cinfo(node, g_test_name,
                                               g_package_name_url);
  EXPECT_TRUE(cinfo.isCalibrated());

  sensor_msgs::CameraInfo ci(cinfo.getCameraInfo());
  sensor_msgs::CameraInfo exp(expected_calibration());
  compare_calibration(exp, ci);
}

// Test load of unresolved "package:" URL files
TEST(GetInfo, unresolvedLoads)
{
  ros::NodeHandle node;
  camera_info_manager::CameraInfoManager cinfo(node);

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("package://")));
  EXPECT_FALSE(cinfo.isCalibrated());

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("package:///calibration.yaml")));
  EXPECT_FALSE(cinfo.isCalibrated());

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("package://no_such_package/calibration.yaml")));
  EXPECT_FALSE(cinfo.isCalibrated());

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("package://camera_info_manager/no_such_file.yaml")));
  EXPECT_FALSE(cinfo.isCalibrated());
}

// Test load of "package:" URL after changing name
TEST(GetInfo, nameChange)
{
  ros::NodeHandle node;
  const std::string missing_file("no_such_file");

  // first declare using non-existent camera name
  camera_info_manager::CameraInfoManager cinfo(node, missing_file,
                                               g_package_name_url);
  EXPECT_FALSE(cinfo.isCalibrated());

  // set name so it resolves to a test file that does exist
  EXPECT_TRUE(cinfo.setCameraName(g_test_name));
  EXPECT_TRUE(cinfo.isCalibrated());
  sensor_msgs::CameraInfo ci(cinfo.getCameraInfo());
  sensor_msgs::CameraInfo exp(expected_calibration());
  compare_calibration(exp, ci);
}

// Test load of invalid CameraInfo URLs
TEST(GetInfo, invalidLoads)
{
  ros::NodeHandle node;
  camera_info_manager::CameraInfoManager cinfo(node);

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("flash:///")));
  EXPECT_FALSE(cinfo.isCalibrated());

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("html://ros.org/wiki/camera_info_manager")));
  EXPECT_FALSE(cinfo.isCalibrated());

  sensor_msgs::CameraInfo ci(cinfo.getCameraInfo());
  sensor_msgs::CameraInfo exp;
  compare_calibration(exp, ci);
}

// Test ability to set CameraInfo directly
TEST(SetInfo, setCameraInfo)
{
  ros::NodeHandle node;
  camera_info_manager::CameraInfoManager cinfo(node);

  // issue calibration service request
  sensor_msgs::CameraInfo exp(expected_calibration());
  bool success = cinfo.setCameraInfo(exp);
  EXPECT_TRUE(success);

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success)
    {
      // check that it worked
      EXPECT_TRUE(cinfo.isCalibrated());
      sensor_msgs::CameraInfo ci = cinfo.getCameraInfo();
      compare_calibration(exp, ci);
    }
}

// Test ability to set calibrated CameraInfo
TEST(SetInfo, setCalibration)
{
  ros::NodeHandle node;
  camera_info_manager::CameraInfoManager cinfo(node);

  // issue calibration service request
  sensor_msgs::CameraInfo exp(expected_calibration());
  bool success = set_calibration(node, exp);

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success)
    {
      // check that it worked
      EXPECT_TRUE(cinfo.isCalibrated());
      sensor_msgs::CameraInfo ci = cinfo.getCameraInfo();
      compare_calibration(exp, ci);
    }
}

// Test ability to save calibrated CameraInfo in default URL
TEST(SetInfo, saveCalibrationDefault)
{
  ros::NodeHandle node;
  sensor_msgs::CameraInfo exp(expected_calibration());
  bool success;

  // Set ${ROS_HOME} to /tmp, then delete the /tmp/camera_info
  // directory and everything in it.
  setenv("ROS_HOME", "/tmp", true);
  delete_tmp_camera_info_directory();

  {
    // create instance to save calibrated data
    camera_info_manager::CameraInfoManager cinfo(node);
    EXPECT_FALSE(cinfo.isCalibrated());

    // issue calibration service request
    success = set_calibration(node, exp);
    EXPECT_TRUE(cinfo.isCalibrated());
  }

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success)
    {
      // create a new instance to load saved calibration
      camera_info_manager::CameraInfoManager cinfo2(node);
      EXPECT_TRUE(cinfo2.isCalibrated());
      if (cinfo2.isCalibrated())
        {
          sensor_msgs::CameraInfo ci(cinfo2.getCameraInfo());
          compare_calibration(exp, ci);
        }
    }
}

// Test ability to save calibrated CameraInfo to default location with
// explicit camera name
TEST(SetInfo, saveCalibrationCameraName)
{
  ros::NodeHandle node;
  sensor_msgs::CameraInfo exp(expected_calibration());
  bool success;

  // set ${ROS_HOME} to /tmp, delete the calibration file
  std::string ros_home("/tmp");
  setenv("ROS_HOME", ros_home.c_str(), true);
  std::string tmpFile(ros_home + "/camera_info/" + g_camera_name + ".yaml");
  delete_file(tmpFile);

  {
    // create instance to save calibrated data
    camera_info_manager::CameraInfoManager cinfo(node, g_camera_name);
    success = set_calibration(node, exp);
    EXPECT_TRUE(cinfo.isCalibrated());
  }

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success)
    {
      // create a new instance to load saved calibration
      camera_info_manager::CameraInfoManager cinfo2(node);
      std::string url = "file://" + tmpFile;
      cinfo2.loadCameraInfo(std::string(url));
      EXPECT_TRUE(cinfo2.isCalibrated());
      if (cinfo2.isCalibrated())
        {
          sensor_msgs::CameraInfo ci(cinfo2.getCameraInfo());
          compare_calibration(exp, ci);
        }
    }
}

// Test ability to save calibrated CameraInfo in a file
TEST(SetInfo, saveCalibrationFile)
{
  return;

  ros::NodeHandle node;
  sensor_msgs::CameraInfo exp(expected_calibration());
  std::string cname("camera");
  std::string tmpFile("/tmp/camera_info_manager_calibration_test.yaml");
  std::string url = "file://" + tmpFile;
  bool success;

  // first, delete the file
  delete_file(tmpFile);

  {
    // create instance to save calibrated data
    camera_info_manager::CameraInfoManager cinfo(node, cname, url);
    success = set_calibration(node, exp);
    EXPECT_TRUE(cinfo.isCalibrated());
  }

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success)
    {
      // create a new instance to load saved calibration
      camera_info_manager::CameraInfoManager cinfo2(node, cname, url);
      EXPECT_TRUE(cinfo2.isCalibrated());
      if (cinfo2.isCalibrated())
        {
          sensor_msgs::CameraInfo ci(cinfo2.getCameraInfo());
          compare_calibration(exp, ci);
        }
    }
}

// Test ability to save calibrated CameraInfo in a package
// (needs write access).
TEST(SetInfo, saveCalibrationPackage)
{
  ros::NodeHandle node;
  sensor_msgs::CameraInfo exp(expected_calibration());
  std::string pkgPath(ros::package::getPath(g_package_name));
  std::string filename(pkgPath + g_package_filename);
  bool success;

  // first, delete the file
  delete_file(filename);

  {
    // create instance to save calibrated data
    camera_info_manager::CameraInfoManager cinfo(node, g_camera_name,
                                                 g_package_url);
    success = set_calibration(node, exp);
    EXPECT_TRUE(cinfo.isCalibrated());
  }

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success)
    {
      // create a new instance to load saved calibration
      camera_info_manager::CameraInfoManager cinfo2(node, g_camera_name,
                                                    g_package_url);
      EXPECT_TRUE(cinfo2.isCalibrated());
      if (cinfo2.isCalibrated())
        {
          sensor_msgs::CameraInfo ci(cinfo2.getCameraInfo());
          compare_calibration(exp, ci);
        }
    }
}

TEST(UrlSubstitution, cameraName)
{
  ros::NodeHandle node;
  std::string name_url;
  std::string exp_url;

  // resolve a GUID camera name
  name_url = "package://" + g_package_name + "/tests/${NAME}.yaml";
  exp_url = "package://" + g_package_name + "/tests/" + g_camera_name + ".yaml";
  check_url_substitution(node, name_url, exp_url, g_camera_name);

  // substitute camera name "test"
  name_url = "package://" + g_package_name + "/tests/${NAME}_calibration.yaml";
  std::string test_name("test");
  exp_url = "package://" + g_package_name + "/tests/" + test_name
    + "_calibration.yaml";
  check_url_substitution(node, name_url, exp_url, test_name);

  // with an '_' in the name
  test_name = "camera_1024x768";
  exp_url = "package://" + g_package_name + "/tests/" + test_name
    + "_calibration.yaml";
  check_url_substitution(node, name_url, exp_url, test_name);

  // substitute empty camera name
  name_url = "package://" + g_package_name + "/tests/${NAME}_calibration.yaml";
  std::string empty_name("");
  exp_url = "package://" + g_package_name + "/tests/" + empty_name
    + "_calibration.yaml";
  check_url_substitution(node, name_url, exp_url, empty_name);

  // substitute test camera calibration from this package
  check_url_substitution(node, g_package_name_url, g_package_url, g_test_name);
}

TEST(UrlSubstitution, rosHome)
{
  ros::NodeHandle node;
  std::string name_url;
  std::string exp_url;
  char *home_env = getenv("HOME");
  std::string home(home_env);

  // resolve ${ROS_HOME} with environment variable undefined
  unsetenv("ROS_HOME");
  name_url = "file://${ROS_HOME}/camera_info/test_camera.yaml";
  exp_url = "file://" + home + "/.ros/camera_info/test_camera.yaml";
  check_url_substitution(node, name_url, exp_url, g_camera_name);

  // resolve ${ROS_HOME} with environment variable defined
  setenv("ROS_HOME", "/my/ros/home", true);
  name_url = "file://${ROS_HOME}/camera_info/test_camera.yaml";
  exp_url = "file:///my/ros/home/camera_info/test_camera.yaml";
  check_url_substitution(node, name_url, exp_url, g_camera_name);
}

TEST(UrlSubstitution, unmatchedDollarSigns)
{
  ros::NodeHandle node;

  // test for "$$" in the URL (NAME should be resolved)
  std::string name_url("file:///tmp/$${NAME}.yaml");
  std::string exp_url("file:///tmp/$" + g_camera_name + ".yaml");
  check_url_substitution(node, name_url, exp_url, g_camera_name);

  // test for "$" in middle of string
  name_url = "file:///$whatever.yaml";
  check_url_substitution(node, name_url, name_url, g_camera_name);

  // test for "$$" in middle of string
  name_url = "file:///something$$whatever.yaml";
  check_url_substitution(node, name_url, name_url, g_camera_name);

  // test for "$$" at end of string
  name_url = "file:///$$";
  check_url_substitution(node, name_url, name_url, g_camera_name);
}

TEST(UrlSubstitution, emptyURL)
{
  // test that empty URL is handled correctly
  ros::NodeHandle node;
  std::string empty_url("");
  check_url_substitution(node, empty_url, empty_url, g_camera_name);
}

TEST(UrlSubstitution, invalidVariables)
{
  ros::NodeHandle node;
  std::string name_url;

  // missing "{...}"
  name_url = "file:///tmp/$NAME.yaml";
  check_url_substitution(node, name_url, name_url, g_camera_name);

  // invalid substitution variable name
  name_url = "file:///tmp/${INVALID}/calibration.yaml";
  check_url_substitution(node, name_url, name_url, g_camera_name);

  // truncated substitution variable
  name_url = "file:///tmp/${NAME";
  check_url_substitution(node, name_url, name_url, g_camera_name);

  // missing substitution variable
  name_url = "file:///tmp/${}";
  check_url_substitution(node, name_url, name_url, g_camera_name);

  // no exception thrown for single "$" at end of string
  name_url = "file:///$";
  check_url_substitution(node, name_url, name_url, g_camera_name);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_info_manager_unit_test");
  testing::InitGoogleTest(&argc, argv);

  // create asynchronous thread for handling service requests
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // run the tests in this thread
  return RUN_ALL_TESTS();
}
