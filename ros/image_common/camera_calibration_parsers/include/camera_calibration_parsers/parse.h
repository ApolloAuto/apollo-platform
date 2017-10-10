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

#ifndef CAMERA_CALIBRATION_PARSERS_PARSE_H
#define CAMERA_CALIBRATION_PARSERS_PARSE_H

#include <string>
#include <sensor_msgs/CameraInfo.h>

/// @todo: use stream-based API, so no read/parse distinction
namespace camera_calibration_parsers {

/**
 * \brief Write calibration parameters to a file.
 *
 * The file name extension (.yml, .yaml, or .ini) determines the output format.
 *
 * \param file_name File to write
 * \param camera_name Name of the camera
 * \param cam_info Camera parameters
 */
bool writeCalibration(const std::string& file_name, const std::string& camera_name,
                      const sensor_msgs::CameraInfo& cam_info);

/**
 * \brief Read calibration parameters from a file.
 *
 * The file may be YAML or INI format.
 *
 * \param file_name File to read
 * \param[out] camera_name Name of the camera
 * \param[out] cam_info Camera parameters
 */
bool readCalibration(const std::string& file_name, std::string& camera_name,
                     sensor_msgs::CameraInfo& cam_info);

/**
 * \brief Parse calibration parameters from a string in memory.
 *
 * \param buffer Calibration string
 * \param format Format of calibration string, "yml" or "ini"
 * \param[out] camera_name Name of the camera
 * \param[out] cam_info Camera parameters
 */
bool parseCalibration(const std::string& buffer, const std::string& format,
                      std::string& camera_name, sensor_msgs::CameraInfo& cam_info);

} //namespace camera_calibration_parsers

#endif
