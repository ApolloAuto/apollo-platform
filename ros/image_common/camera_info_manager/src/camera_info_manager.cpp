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

#include <string>
#include <locale>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <camera_calibration_parsers/parse.h>

#include "camera_info_manager/camera_info_manager.h"

/** @file

    @brief CameraInfo Manager implementation

    Provides CameraInfo, handles the SetCameraInfo service requests,
    saves and restores sensor_msgs/CameraInfo data.

    @author Jack O'Quin
 */

namespace camera_info_manager
{

using namespace camera_calibration_parsers;

/** URL to use when no other is defined. */
const std::string
  default_camera_info_url = "file://${ROS_HOME}/camera_info/${NAME}.yaml";

/** Constructor
 *
 * @param nh node handle, normally for the driver's streaming name
 *           space ("camera").  The service name is relative to this
 *           handle.  Nodes supporting multiple cameras may use
 *           subordinate names, like "left/camera" and "right/camera".
 * @param cname default camera name
 * @param url default Uniform Resource Locator for loading and saving data.
 */
CameraInfoManager::CameraInfoManager(ros::NodeHandle nh,
                                     const std::string &cname,
                                     const std::string &url):
  nh_(nh),
  camera_name_(cname),
  url_(url),
  loaded_cam_info_(false)
{
  // register callback for camera calibration service request
  info_service_ = nh_.advertiseService("set_camera_info",
                                       &CameraInfoManager::setCameraInfoService, this);
}

/** Get the current CameraInfo data.
 *
 * If CameraInfo has not yet been loaded, an attempt must be made
 * here.  To avoid that, ensure that loadCameraInfo() ran previously.
 * If the load is attempted but fails, an empty CameraInfo will be
 * supplied.
 *
 * The matrices are all zeros if no calibration is available. The
 * image pipeline handles that as uncalibrated data.
 *
 * @warning The caller @em must fill in the message Header of the
 *          CameraInfo returned.  The time stamp and frame_id should
 *          normally be the same as the corresponding Image message
 *          Header fields.
 */
sensor_msgs::CameraInfo CameraInfoManager::getCameraInfo(void)
{
  while (ros::ok())
    {
      std::string cname;
      std::string url;
      {
        boost::mutex::scoped_lock lock_(mutex_);
        if (loaded_cam_info_)
          {
            return cam_info_;           // all done
          }

        // load being attempted now
        loaded_cam_info_ = true;

        // copy the name and URL strings
        url = url_;
        cname = camera_name_;

      } // release the lock

      // attempt load without the lock, it is not recursive
      loadCalibration(url, cname);
    }

  return sensor_msgs::CameraInfo();
}

/** Get file name corresponding to a @c package: URL.
 *
 * @param url a copy of the Uniform Resource Locator
 * @return file name if package found, "" otherwise
 */
std::string CameraInfoManager::getPackageFileName(const std::string &url)
{
  ROS_DEBUG_STREAM("camera calibration URL: " << url);

  // Scan URL from after "package://" until next '/' and extract
  // package name.  The parseURL() already checked that it's present.
  size_t prefix_len = std::string("package://").length();
  size_t rest = url.find('/', prefix_len);
  std::string package(url.substr(prefix_len, rest - prefix_len));

  // Look up the ROS package path name.
  std::string pkgPath(ros::package::getPath(package));
  if (pkgPath.empty())                  // package not found?
    {
      ROS_WARN_STREAM("unknown package: " << package << " (ignored)");
      return pkgPath;
    }
  else
    {
      // Construct file name from package location and remainder of URL.
      return pkgPath + url.substr(rest);
    }
}

/** Is the current CameraInfo calibrated?
 *
 * If CameraInfo has not yet been loaded, an attempt must be made
 * here.  To avoid that, ensure that loadCameraInfo() ran previously.
 * If the load failed, CameraInfo will be empty and this predicate
 * will return false.
 *
 * @return true if the current CameraInfo is calibrated.
 */
bool CameraInfoManager::isCalibrated(void)
{
  while (true)
    {
      std::string cname;
      std::string url;
      {
        boost::mutex::scoped_lock lock_(mutex_);
        if (loaded_cam_info_)
          {
            return (cam_info_.K[0] != 0.0);
          }

        // load being attempted now
        loaded_cam_info_ = true;

        // copy the name and URL strings
        url = url_;
        cname = camera_name_;

      } // release the lock

      // attempt load without the lock, it is not recursive
      loadCalibration(url, cname);
    }
}

/** Load CameraInfo calibration data (if any).
 *
 * @pre mutex_ unlocked
 *
 * @param url a copy of the Uniform Resource Locator
 * @param cname is a copy of the camera_name_
 * @return true if URL contains calibration data.
 *
 * sets cam_info_, if successful
 */
bool CameraInfoManager::loadCalibration(const std::string &url,
                                        const std::string &cname)
{
  bool success = false;                 // return value

  const std::string resURL(resolveURL(url, cname));
  url_type_t url_type = parseURL(resURL);

  if (url_type != URL_empty)
    {
      ROS_INFO_STREAM("camera calibration URL: " << resURL);
    }

  switch (url_type)
    {
    case URL_empty:
      {
        ROS_INFO("using default calibration URL");
        success = loadCalibration(default_camera_info_url, cname);
        break;
      }
    case URL_file:
      {
        success = loadCalibrationFile(resURL.substr(7), cname);
        break;
      }
    case URL_flash:
      {
        ROS_WARN("[CameraInfoManager] reading from flash not implemented yet");
        break;
      }
    case URL_package:
      {
        std::string filename(getPackageFileName(resURL));
        if (!filename.empty())
          success = loadCalibrationFile(filename, cname);
        break;
      }
    default:
      {
        ROS_ERROR_STREAM("Invalid camera calibration URL: " << resURL);
        break;
      }
    }

  return success;
}

/** Load CameraInfo calibration data from a file.
 *
 * @pre mutex_ unlocked
 *
 * @param filename containing CameraInfo to read
 * @param cname is a copy of the camera_name_
 * @return true if URL contains calibration data.
 *
 * Sets cam_info_, if successful
 */
bool CameraInfoManager::loadCalibrationFile(const std::string &filename,
                                            const std::string &cname)
{
  bool success = false;

  ROS_DEBUG_STREAM("reading camera calibration from " << filename);
  std::string cam_name;
  sensor_msgs::CameraInfo cam_info;

  if (readCalibration(filename, cam_name, cam_info))
    {
      if (cname != cam_name)
        {
          ROS_WARN_STREAM("[" << cname << "] does not match name "
                          << cam_name << " in file " << filename);
        }
      success = true;
      {
        // lock only while updating cam_info_
        boost::mutex::scoped_lock lock(mutex_);
        cam_info_ = cam_info;
      }
    }
  else
    {
      ROS_WARN_STREAM("Camera calibration file " << filename << " not found.");
    }

  return success;
}

/** Set a new URL and load its calibration data (if any).
 *
 * If multiple threads call this method simultaneously with different
 * URLs, there is no guarantee which will prevail.
 *
 * @param url new Uniform Resource Locator for CameraInfo.
 * @return true if new URL contains calibration data.
 *
 * @post @c loaded_cam_info_ true (meaning a load was attempted, even
 *       if it failed); @c cam_info_ updated, if successful.
 */
bool CameraInfoManager::loadCameraInfo(const std::string &url)
{
  std::string cname;
  {
    boost::mutex::scoped_lock lock(mutex_);
    url_ = url;
    cname = camera_name_;
    loaded_cam_info_ = true;
  }

  // load using copies of the parameters, no need to hold the lock
  return loadCalibration(url, cname);
}


/** Resolve Uniform Resource Locator string.
 *
 * @param url a copy of the Uniform Resource Locator, which may
 *            include <tt>${...}</tt> substitution variables.
 * @param cname is a copy of the camera_name_
 *
 * @return a copy of the URL with any variable information resolved.
 */
std::string CameraInfoManager::resolveURL(const std::string &url,
                                          const std::string &cname)
{
  std::string resolved;
  size_t rest = 0;

  while (true)
    {
      // find the next '$' in the URL string
      size_t dollar  = url.find('$', rest);

      if (dollar >= url.length())
        {
          // no more variables left in the URL
          resolved += url.substr(rest);
          break;
        }

      // copy characters up to the next '$'
      resolved += url.substr(rest, dollar-rest);

      if (url.substr(dollar+1, 1) != "{")
        {
          // no '{' follows, so keep the '$'
          resolved += "$";
        }
      else if (url.substr(dollar+1, 6) == "{NAME}")
        {
          // substitute camera name
          resolved += cname;
          dollar += 6;
        }
      else if (url.substr(dollar+1, 10) == "{ROS_HOME}")
        {
          // substitute $ROS_HOME
          std::string ros_home;
          char *ros_home_env;
          if ((ros_home_env = getenv("ROS_HOME")))
            {
              // use environment variable
              ros_home = ros_home_env;
            }
          else if ((ros_home_env = getenv("HOME")))
            {
              // use "$HOME/.ros"
              ros_home = ros_home_env;
              ros_home += "/.ros";
            }
          resolved += ros_home;
          dollar += 10;
        }
      else
        {
          // not a valid substitution variable
          ROS_ERROR_STREAM("[CameraInfoManager]"
                           " invalid URL substitution (not resolved): "
                           << url);
          resolved += "$";            // keep the bogus '$'
        }

      // look for next '$'
      rest = dollar + 1;
    }

  return resolved;
}

/** Parse calibration Uniform Resource Locator.
 *
 * @param url string to parse
 * @return URL type
 *
 * @note Recognized but unsupported URL types have enum values >= URL_invalid.
 */
CameraInfoManager::url_type_t CameraInfoManager::parseURL(const std::string &url)
{
  if (url == "")
    {
      return URL_empty;
    }
  if (boost::iequals(url.substr(0, 8), "file:///"))
    {
      return URL_file;
    }
  if (boost::iequals(url.substr(0, 9), "flash:///"))
    {
      return URL_flash;
    }
  if (boost::iequals(url.substr(0, 10), "package://"))
    {
      // look for a '/' following the package name, make sure it is
      // there, the name is not empty, and something follows it
      size_t rest = url.find('/', 10);
      if (rest < url.length()-1 && rest > 10)
        return URL_package;
    }
  return URL_invalid;
}

/** Save CameraInfo calibration data.
 *
 * @pre mutex_ unlocked
 *
 * @param new_info contains CameraInfo to save
 * @param url is a copy of the URL storage location (if empty, use
 *            @c file://${ROS_HOME}/camera_info/${NAME}.yaml)
 * @param cname is a copy of the camera_name_
 * @return true, if successful
 */
bool
CameraInfoManager::saveCalibration(const sensor_msgs::CameraInfo &new_info,
                                   const std::string &url,
                                   const std::string &cname)
{
  bool success = false;

  const std::string resURL(resolveURL(url, cname));

  switch (parseURL(resURL))
    {
    case URL_empty:
      {
        // store using default file name
        success = saveCalibration(new_info, default_camera_info_url, cname);
        break;
      }
    case URL_file:
      {
        success = saveCalibrationFile(new_info, resURL.substr(7), cname);
        break;
      }
    case URL_package:
      {
        std::string filename(getPackageFileName(resURL));
        if (!filename.empty())
          success = saveCalibrationFile(new_info, filename, cname);
        break;
      }
    default:
      {
        // invalid URL, save to default location
        ROS_ERROR_STREAM("invalid url: " << resURL << " (ignored)");
        success = saveCalibration(new_info, default_camera_info_url, cname);
        break;
      }
    }

  return success;
}
  
/** Save CameraInfo calibration data to a file.
 *
 * @pre mutex_ unlocked
 *
 * @param new_info contains CameraInfo to save
 * @param filename is local file to store data
 * @param cname is a copy of the camera_name_
 * @return true, if successful
 */
bool
CameraInfoManager::saveCalibrationFile(const sensor_msgs::CameraInfo &new_info,
                                       const std::string &filename,
                                       const std::string &cname)
{
  ROS_INFO_STREAM("writing calibration data to " << filename);

  // isolate the name of the containing directory
  size_t last_slash = filename.rfind('/');
  if (last_slash >= filename.length())
    {
      // No slash in the name.  This should never happen, the URL
      // parser ensures there is at least one '/' at the beginning.
      ROS_ERROR_STREAM("filename [" << filename << "] has no '/'");
      return false;                     // not a valid URL
    }

  // make sure the directory exists and is writable
  std::string dirname(filename.substr(0, last_slash+1));
  struct stat stat_data;
  int rc = stat(dirname.c_str(), &stat_data);
  if (rc != 0)
    {
      if (errno == ENOENT)
        {
          // directory does not exist, try to create it and its parents
          std::string command("mkdir -p " + dirname);
          rc = system(command.c_str());
          if (rc != 0)
            {
              // mkdir failed
              ROS_ERROR_STREAM("unable to create path to directory ["
                               << dirname << "]");
              return false;
            }
        }
      else
        {
          // not accessible, or something screwy
          ROS_ERROR_STREAM("directory [" << dirname << "] not accessible");
          return false;
        }
    }
  else if (!S_ISDIR(stat_data.st_mode))
    {
      // dirname exists but is not a directory
      ROS_ERROR_STREAM("[" << dirname << "] is not a directory");
      return false;
    }

  // Directory exists and is accessible. Permissions might still be bad.

  // Currently, writeCalibration() always returns true no matter what
  // (ros-pkg ticket #5010).
  return writeCalibration(filename, cname, new_info);
}

/** Callback for SetCameraInfo request.
 *
 * Always updates cam_info_ class variable, even if save fails.
 *
 * @param req SetCameraInfo request message
 * @param rsp SetCameraInfo response message
 * @return true if message handled
 */
bool 
CameraInfoManager::setCameraInfoService(sensor_msgs::SetCameraInfo::Request &req,
                                        sensor_msgs::SetCameraInfo::Response &rsp)
{
  // copies of class variables needed for saving calibration
  std::string url_copy;
  std::string cname;
  {
    boost::mutex::scoped_lock lock(mutex_);
    cam_info_ = req.camera_info;
    url_copy = url_;
    cname = camera_name_;
    loaded_cam_info_ = true;
  }

  if (!nh_.ok())
    {
      ROS_ERROR("set_camera_info service called, but driver not running.");
      rsp.status_message = "Camera driver not running.";
      rsp.success = false;
      return false;
    }

  rsp.success = saveCalibration(req.camera_info, url_copy, cname);
  if (!rsp.success)
    rsp.status_message = "Error storing camera calibration.";

  return true;
}

/** Set a new camera name.
 *
 * @param cname new camera name to use for saving calibration data
 *
 * @return true if new name has valid syntax; valid names contain only
 *              alphabetic, numeric, or '_' characters.
 *
 * @post @c cam_name_ updated, if valid; since it may affect the URL,
 *       @c cam_info_ will be reloaded before being used again.
 */
bool CameraInfoManager::setCameraName(const std::string &cname)
{
  // the camera name may not be empty
  if (cname.empty())
    return false;

  // validate the camera name characters
  for (unsigned i = 0; i < cname.size(); ++i)
    {
      if (!isalnum(cname[i]) && cname[i] != '_')
        return false;
    }

  // The name is valid, so update our private copy.  Since the new
  // name might cause the existing URL to resolve somewhere else,
  // force @c cam_info_ to be reloaded before being used again.
  {
    boost::mutex::scoped_lock lock(mutex_);
    camera_name_ = cname;
    loaded_cam_info_ = false;
  }

  return true;
}

/** Set the camera info manually
 *
 * @param camera_info new camera calibration data
 *
 * @return true if new camera info is set
 *
 * @post @c cam_info_ updated, if valid;
 */
bool CameraInfoManager::setCameraInfo(const sensor_msgs::CameraInfo &camera_info)
{
  boost::mutex::scoped_lock lock(mutex_);

  cam_info_ = camera_info;
  loaded_cam_info_ = true;

  return true;
}

/** Validate URL syntax.
 *
 * @param url Uniform Resource Locator to check
 *
 * @return true if URL syntax is supported by CameraInfoManager
 *              (although the resource need not actually exist)
 */
bool CameraInfoManager::validateURL(const std::string &url)
{
  std::string cname;                    // copy of camera name
  {
    boost::mutex::scoped_lock lock(mutex_);
    cname = camera_name_;
  } // release the lock

  url_type_t url_type = parseURL(resolveURL(url, cname));
  return (url_type < URL_invalid);
}

} // namespace camera_info_manager
