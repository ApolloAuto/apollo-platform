#ifndef IMAGE_GEOMETRY_STEREO_CAMERA_MODEL_H
#define IMAGE_GEOMETRY_STEREO_CAMERA_MODEL_H

#include "image_geometry/pinhole_camera_model.h"

namespace image_geometry {

/**
 * \brief Simplifies interpreting stereo image pairs geometrically using the
 * parameters from the left and right sensor_msgs/CameraInfo.
 */
class StereoCameraModel
{
public:
  StereoCameraModel();

  StereoCameraModel(const StereoCameraModel& other);

  StereoCameraModel& operator=(const StereoCameraModel& other);

  /**
   * \brief Set the camera parameters from the sensor_msgs/CameraInfo messages.
   */
  bool fromCameraInfo(const sensor_msgs::CameraInfo& left,
                      const sensor_msgs::CameraInfo& right);

  /**
   * \brief Set the camera parameters from the sensor_msgs/CameraInfo messages.
   */
  bool fromCameraInfo(const sensor_msgs::CameraInfoConstPtr& left,
                      const sensor_msgs::CameraInfoConstPtr& right);

  /**
   * \brief Get the left monocular camera model.
   */
  const PinholeCameraModel& left() const;

  /**
   * \brief Get the right monocular camera model.
   */
  const PinholeCameraModel& right() const;

  /**
   * \brief Get the name of the camera coordinate frame in tf.
   *
   * For stereo cameras, both the left and right CameraInfo should be in the left
   * optical frame.
   */
  std::string tfFrame() const;

  /**
   * \brief Project a rectified pixel with disparity to a 3d point.
   */
  void projectDisparityTo3d(const cv::Point2d& left_uv_rect, float disparity, cv::Point3d& xyz) const;

  /**
   * \brief Project a disparity image to a 3d point cloud.
   *
   * If handleMissingValues = true, all points with minimal disparity (outliers) have
   * Z set to MISSING_Z (currently 10000.0).
   */
  void projectDisparityImageTo3d(const cv::Mat& disparity, cv::Mat& point_cloud,
                                 bool handleMissingValues = false) const;
  static const double MISSING_Z;
  
  /**
   * \brief Returns the disparity reprojection matrix.
   */
  const cv::Matx44d& reprojectionMatrix() const;

  /**
   * \brief Returns the horizontal baseline in world coordinates.
   */
  double baseline() const;

  /**
   * \brief Returns the depth at which a point is observed with a given disparity.
   *
   * This is the inverse of getDisparity().
   */
  double getZ(double disparity) const;

  /**
   * \brief Returns the disparity observed for a point at depth Z.
   *
   * This is the inverse of getZ().
   */
  double getDisparity(double Z) const;

  /**
   * \brief Returns true if the camera has been initialized
   */
  bool initialized() const { return left_.initialized() && right_.initialized(); }
protected:
  PinholeCameraModel left_, right_;
  cv::Matx44d Q_;

  void updateQ();
};


/* Trivial inline functions */
inline const PinholeCameraModel& StereoCameraModel::left() const  { return left_; }
inline const PinholeCameraModel& StereoCameraModel::right() const { return right_; }

inline std::string StereoCameraModel::tfFrame() const { return left_.tfFrame(); }

inline const cv::Matx44d& StereoCameraModel::reprojectionMatrix() const { return Q_; }

inline double StereoCameraModel::baseline() const
{
  /// @todo Currently assuming horizontal baseline
  return -right_.Tx() / right_.fx();
}

inline double StereoCameraModel::getZ(double disparity) const
{
  assert( initialized() );
  return -right_.Tx() / (disparity - (left().cx() - right().cx()));
}

inline double StereoCameraModel::getDisparity(double Z) const
{
  assert( initialized() );
  return -right_.Tx() / Z + (left().cx() - right().cx()); ;
}

} //namespace image_geometry

#endif
