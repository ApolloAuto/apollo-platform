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

#include "camera_calibration_parsers/parse_yml.h"
#include <sensor_msgs/distortion_models.h>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ctime>
#include <cassert>
#include <cstring>
#include <ros/console.h>

namespace camera_calibration_parsers {

/// \cond

static const char CAM_YML_NAME[]    = "camera_name";
static const char WIDTH_YML_NAME[]  = "image_width";
static const char HEIGHT_YML_NAME[] = "image_height";
static const char K_YML_NAME[]      = "camera_matrix";
static const char D_YML_NAME[]      = "distortion_coefficients";
static const char R_YML_NAME[]      = "rectification_matrix";
static const char P_YML_NAME[]      = "projection_matrix";
static const char DMODEL_YML_NAME[] = "distortion_model";

struct SimpleMatrix
{
  int rows;
  int cols;
  double* data;

  SimpleMatrix(int rows, int cols, double* data)
    : rows(rows), cols(cols), data(data)
  {}
};

YAML::Emitter& operator << (YAML::Emitter& out, const SimpleMatrix& m)
{
  out << YAML::BeginMap;
  out << YAML::Key << "rows" << YAML::Value << m.rows;
  out << YAML::Key << "cols" << YAML::Value << m.cols;
  //out << YAML::Key << "dt"   << YAML::Value << "d"; // OpenCV data type specifier
  out << YAML::Key << "data" << YAML::Value;
  out << YAML::Flow;
  out << YAML::BeginSeq;
  for (int i = 0; i < m.rows*m.cols; ++i)
    out << m.data[i];
  out << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}

#ifdef HAVE_NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

void operator >> (const YAML::Node& node, SimpleMatrix& m)
{
  int rows, cols;
  node["rows"] >> rows;
  assert(rows == m.rows);
  node["cols"] >> cols;
  assert(cols == m.cols);
  const YAML::Node& data = node["data"];
  for (int i = 0; i < rows*cols; ++i)
    data[i] >> m.data[i];
}

/// \endcond

bool writeCalibrationYml(std::ostream& out, const std::string& camera_name,
                         const sensor_msgs::CameraInfo& cam_info)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

#if 0
  // Calibration time
  /// @todo Emitting the time breaks yaml-cpp on reading for some reason
  time_t raw_time;
  time( &raw_time );
  emitter << YAML::Key << "calibration_time";
  emitter << YAML::Value << asctime(localtime(&raw_time));
#endif

  // Image dimensions
  emitter << YAML::Key << WIDTH_YML_NAME << YAML::Value << (int)cam_info.width;
  emitter << YAML::Key << HEIGHT_YML_NAME << YAML::Value << (int)cam_info.height;
  
  // Camera name and intrinsics
  emitter << YAML::Key << CAM_YML_NAME << YAML::Value << camera_name;
  emitter << YAML::Key << K_YML_NAME << YAML::Value << SimpleMatrix(3, 3, const_cast<double*>(&cam_info.K[0]));
  emitter << YAML::Key << DMODEL_YML_NAME << YAML::Value << cam_info.distortion_model;
  emitter << YAML::Key << D_YML_NAME << YAML::Value << SimpleMatrix(1, cam_info.D.size(),
                                                                    const_cast<double*>(&cam_info.D[0]));
  emitter << YAML::Key << R_YML_NAME << YAML::Value << SimpleMatrix(3, 3, const_cast<double*>(&cam_info.R[0]));
  emitter << YAML::Key << P_YML_NAME << YAML::Value << SimpleMatrix(3, 4, const_cast<double*>(&cam_info.P[0]));

  emitter << YAML::EndMap;

  out << emitter.c_str();
  return true;
}

bool writeCalibrationYml(const std::string& file_name, const std::string& camera_name,
                         const sensor_msgs::CameraInfo& cam_info)
{
  boost::filesystem::path dir(boost::filesystem::path(file_name).parent_path());
  if (!dir.empty() && !boost::filesystem::exists(dir) &&
     !boost::filesystem::create_directories(dir)){
    ROS_ERROR("Unable to create directory for camera calibration file [%s]", dir.c_str());
  }
  std::ofstream out(file_name.c_str());
  if (!out.is_open())
  {
    ROS_ERROR("Unable to open camera calibration file [%s] for writing", file_name.c_str());
    return false;
  }
  return writeCalibrationYml(out, camera_name, cam_info);
}

bool readCalibrationYml(std::istream& in, std::string& camera_name, sensor_msgs::CameraInfo& cam_info)
{
  try {
#ifdef HAVE_NEW_YAMLCPP
    YAML::Node doc = YAML::Load(in);

    if (doc[CAM_YML_NAME])
      doc[CAM_YML_NAME] >> camera_name;
    else
      camera_name = "unknown";
#else
    YAML::Parser parser(in);
    if (!parser) {
      ROS_ERROR("Unable to create YAML parser for camera calibration");
      return false;
    }
    YAML::Node doc;
    parser.GetNextDocument(doc);

    if (const YAML::Node* name_node = doc.FindValue(CAM_YML_NAME))
      *name_node >> camera_name;
    else
      camera_name = "unknown";
#endif

    doc[WIDTH_YML_NAME] >> cam_info.width;
    doc[HEIGHT_YML_NAME] >> cam_info.height;

    // Read in fixed-size matrices
    SimpleMatrix K_(3, 3, &cam_info.K[0]);
    doc[K_YML_NAME] >> K_;
    SimpleMatrix R_(3, 3, &cam_info.R[0]);
    doc[R_YML_NAME] >> R_;
    SimpleMatrix P_(3, 4, &cam_info.P[0]);
    doc[P_YML_NAME] >> P_;

    // Different distortion models may have different numbers of parameters
#ifdef HAVE_NEW_YAMLCPP
    if (doc[DMODEL_YML_NAME]) {
      doc[DMODEL_YML_NAME] >> cam_info.distortion_model;
    }
#else
    if (const YAML::Node* model_node = doc.FindValue(DMODEL_YML_NAME)) {
      *model_node >> cam_info.distortion_model;
    }
#endif
    else {
      // Assume plumb bob for backwards compatibility
      cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      ROS_WARN("Camera calibration file did not specify distortion model, assuming plumb bob");
    }
    const YAML::Node& D_node = doc[D_YML_NAME];
    int D_rows, D_cols;
    D_node["rows"] >> D_rows;
    D_node["cols"] >> D_cols;
    const YAML::Node& D_data = D_node["data"];
    cam_info.D.resize(D_rows*D_cols);
    for (int i = 0; i < D_rows*D_cols; ++i)
      D_data[i] >> cam_info.D[i];
  
    return true;
  }
  catch (YAML::Exception& e) {
    ROS_ERROR("Exception parsing YAML camera calibration:\n%s", e.what());
    return false;
  }
}

bool readCalibrationYml(const std::string& file_name, std::string& camera_name,
                        sensor_msgs::CameraInfo& cam_info)
{
  std::ifstream fin(file_name.c_str());
  if (!fin.good()) {
    ROS_INFO("Unable to open camera calibration file [%s]", file_name.c_str());
    return false;
  }
  bool success = readCalibrationYml(fin, camera_name, cam_info);
  if (!success)
    ROS_ERROR("Failed to parse camera calibration from file [%s]", file_name.c_str());
  return success;
}

} //namespace camera_calibration_parsers
