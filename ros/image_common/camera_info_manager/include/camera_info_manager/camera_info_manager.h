/* -*- mode: C++ -*- */
/* $Id$ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010-2012 Jack O'Quin
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

#ifndef _CAMERA_INFO_MANAGER_H_
#define _CAMERA_INFO_MANAGER_H_

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

/** @file

    @brief CameraInfo Manager interface

    @author Jack O'Quin
 */

namespace camera_info_manager
{

/** @brief CameraInfo Manager class

    Provides CameraInfo, handles the sensor_msgs/SetCameraInfo service
    requests, saves and restores sensor_msgs/CameraInfo data.

    @par ROS Service

    - @b set_camera_info (sensor_msgs/SetCameraInfo) stores
         calibration information

    Typically, these service requests are made by a calibration
    package, such as:

    - http://www.ros.org/wiki/camera_calibration

    The calling node @em must invoke ros::spin() or ros::spinOnce() in
    some thread, so CameraInfoManager can handle arriving service
    requests.

    @par Camera Name

    The device driver sets a camera name via the
    CameraInfoManager::CameraInfoManager constructor or the
    setCameraName() method.  This name is written when CameraInfo is
    saved, and checked when data are loaded, with a warning logged if
    the name read does not match.

    Syntax: a camera name contains any combination of alphabetic,
            numeric and '_' characters.  Case is significant.

    Camera drivers may use any syntactically valid name they please.
    Where possible, it is best for the name to be unique to the
    device, such as a GUID, or the make, model and serial number.  Any
    parameters that affect calibration, such as resolution, focus,
    zoom, etc., may also be included in the name, uniquely identifying
    each CameraInfo file.

    Beginning with Electric Emys, the camera name can be resolved as
    part of the URL, allowing direct access to device-specific
    calibration information.

    @par Uniform Resource Locator

    The location for getting and saving calibration data is expressed
    by Uniform Resource Locator.  The driver defines a URL via the
    CameraInfoManager::CameraInfoManager constructor or the
    loadCameraInfo() method.  Many drivers provide a @c
    ~camera_info_url parameter so users may customize this URL, but
    that is handled outside this class.

    Typically, cameras store calibration information in a file, which
    can be in any format supported by @c camera_calibration_parsers.
    Currently, that includes YAML and Videre INI files, identified by
    their .yaml or .ini extensions as shown in the examples.  These
    file formats are described here:

    - http://www.ros.org/wiki/camera_calibration_parsers#File_formats

    Example URL syntax:

    - file:///full/path/to/local/file.yaml
    - file:///full/path/to/videre/file.ini
    - package://camera_info_manager/tests/test_calibration.yaml
    - package://ros_package_name/calibrations/camera3.yaml

    The @c file: URL specifies a full path name in the local system.
    The @c package: URL is handled the same as @c file:, except the
    path name is resolved relative to the location of the named ROS
    package, which @em must be reachable via @c $ROS_PACKAGE_PATH.

    Beginning with Electric Emys, the URL may contain substitution
    variables delimited by <tt>${...}</tt>, including:

    - @c ${NAME} resolved to the current camera name defined by the
                 device driver.
    - @c ${ROS_HOME} resolved to the @c $ROS_HOME environment variable
                     if defined, <tt>~/.ros</tt> if not.

    Resolution is done in a single pass through the URL string.
    Variable values containing substitutable strings are not resolved
    recursively.  Unrecognized variable names are treated literally
    with no substitution, but an error is logged.

    Examples with variable substitution:

    - package://my_cameras/calibrations/${NAME}.yaml
    - file://${ROS_HOME}/camera_info/left_front_camera.yaml

    In C-turtle and Diamondback, if the URL was empty, no calibration
    data were loaded, and any data provided via `set_camera_info`
    would be stored in:

    - file:///tmp/calibration_${NAME}.yaml

    Beginning in Electric, the default URL changed to:

    - file://${ROS_HOME}/camera_info/${NAME}.yaml.

    If that file exists, its contents are used. Any new calibration
    will be stored there, missing parent directories being created if
    necessary and possible.

    @par Loading Calibration Data

    Prior to Fuerte, calibration information was loaded in the
    constructor, and again each time the URL or camera name was
    updated. This frequently caused logging of confusing and
    misleading error messages.

    Beginning in Fuerte, camera_info_manager loads nothing until the
    @c loadCameraInfo(), @c isCalibrated() or @c getCameraInfo()
    method is called. That suppresses bogus error messages, but allows
    (valid) load errors to occur during the first @c getCameraInfo(),
    or @c isCalibrated(). To avoid that, do an explicit @c
    loadCameraInfo() first.

*/

class CameraInfoManager
{
 public:

  CameraInfoManager(ros::NodeHandle nh,
                    const std::string &cname="camera",
                    const std::string &url="");

  sensor_msgs::CameraInfo getCameraInfo(void);
  bool isCalibrated(void);
  bool loadCameraInfo(const std::string &url);
  std::string resolveURL(const std::string &url,
                         const std::string &cname);
  bool setCameraName(const std::string &cname);
  bool setCameraInfo(const sensor_msgs::CameraInfo &camera_info);
  bool validateURL(const std::string &url);

 private:

  // recognized URL types
  typedef enum
    {
      // supported URLs
      URL_empty = 0,             // empty string
      URL_file,                  // file:
      URL_package,               // package: 
      // URLs not supported
      URL_invalid,               // anything >= is invalid
      URL_flash,                 // flash: 
    } url_type_t;

  // private methods
  std::string getPackageFileName(const std::string &url);
  bool loadCalibration(const std::string &url,
                       const std::string &cname);
  bool loadCalibrationFile(const std::string &filename,
                           const std::string &cname);
  url_type_t parseURL(const std::string &url);
  bool saveCalibration(const sensor_msgs::CameraInfo &new_info,
                       const std::string &url,
                       const std::string &cname);
  bool saveCalibrationFile(const sensor_msgs::CameraInfo &new_info,
                           const std::string &filename,
                           const std::string &cname);
  bool setCameraInfoService(sensor_msgs::SetCameraInfo::Request &req,
                            sensor_msgs::SetCameraInfo::Response &rsp);

  /** @brief mutual exclusion lock for private data
   *
   *  This non-recursive mutex is only held for a short time while
   *  accessing or changing private class variables.  To avoid
   *  deadlocks and contention, it is never held during I/O or while
   *  invoking a callback.  Most private methods operate on copies of
   *  class variables, keeping the mutex hold time short.
   */
  boost::mutex mutex_;

  // private data
  ros::NodeHandle nh_;                  ///< node handle for service
  ros::ServiceServer info_service_;     ///< set_camera_info service
  std::string camera_name_;             ///< camera name
  std::string url_;                     ///< URL for calibration data
  sensor_msgs::CameraInfo cam_info_;    ///< current CameraInfo
  bool loaded_cam_info_;                ///< cam_info_ load attempted

}; // class CameraInfoManager

}; // namespace camera_info_manager

#endif // _CAMERA_INFO_MANAGER_H_
