/*********************************************************************
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#include "camera_calibration_parsers/parse.h"
#include "camera_calibration_parsers/parse_ini.h"
#include "camera_calibration_parsers/parse_yml.h"

#include <boost/algorithm/string/predicate.hpp>

namespace camera_calibration_parsers {

bool writeCalibration(const std::string& file_name, const std::string& camera_name,
                      const sensor_msgs::CameraInfo& cam_info)
{
  if (boost::iends_with(file_name, ".ini"))
    return writeCalibrationIni(file_name, camera_name, cam_info);
  if (boost::iends_with(file_name, ".yml") || boost::iends_with(file_name, ".yaml"))
    return writeCalibrationYml(file_name, camera_name, cam_info);

  return false;
}

bool readCalibration(const std::string& file_name, std::string& camera_name,
                     sensor_msgs::CameraInfo& cam_info)
{
  if (boost::iends_with(file_name, ".ini"))
    return readCalibrationIni(file_name, camera_name, cam_info);
  if (boost::iends_with(file_name, ".yml") || boost::iends_with(file_name, ".yaml"))
    return readCalibrationYml(file_name, camera_name, cam_info);

  return false;
}

bool parseCalibration(const std::string& buffer, const std::string& format,
                      std::string& camera_name, sensor_msgs::CameraInfo& cam_info)
{
  if (format != "ini")
    return false;

  return parseCalibrationIni(buffer, camera_name, cam_info);
}

} //namespace camera_calibration_parsers
