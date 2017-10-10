#include "image_geometry/stereo_camera_model.h"

namespace image_geometry {

StereoCameraModel::StereoCameraModel()
  : Q_(0.0)
{
  Q_(0,0) = Q_(1,1) = 1.0;
}

StereoCameraModel::StereoCameraModel(const StereoCameraModel& other)
  : left_(other.left_), right_(other.right_),
    Q_(0.0)
{
  Q_(0,0) = Q_(1,1) = 1.0;
  if (other.initialized())
    updateQ();
}

StereoCameraModel& StereoCameraModel::operator=(const StereoCameraModel& other)
{
  if (other.initialized())
    this->fromCameraInfo(other.left_.cameraInfo(), other.right_.cameraInfo());
  return *this;
}

bool StereoCameraModel::fromCameraInfo(const sensor_msgs::CameraInfo& left,
                                       const sensor_msgs::CameraInfo& right)
{
  bool changed_left  = left_.fromCameraInfo(left);
  bool changed_right = right_.fromCameraInfo(right);
  bool changed = changed_left || changed_right;

  // Note: don't require identical time stamps to allow imperfectly synced stereo.
  assert( left_.tfFrame() == right_.tfFrame() );
  assert( left_.fx() == right_.fx() );
  assert( left_.fy() == right_.fy() );
  assert( left_.cy() == right_.cy() );
  // cx may differ for verged cameras

  if (changed)
    updateQ();

  return changed;
}

bool StereoCameraModel::fromCameraInfo(const sensor_msgs::CameraInfoConstPtr& left,
                                       const sensor_msgs::CameraInfoConstPtr& right)
{
  return fromCameraInfo(*left, *right);
}

void StereoCameraModel::updateQ()
{
  // Update variable fields of reprojection matrix
  /*
    From Springer Handbook of Robotics, p. 524:

         [ Fx    0  Cx   0   ]
    P  = [ 0     Fy Cy   0   ]
         [ 0     0  1    0   ]

         [ Fx    0  Cx' FxTx ]
    P' = [ 0     Fy Cy   0    ]
         [ 0     0  1    0    ]
    where primed parameters are from the left projection matrix, unprimed from the right.

    [u   v 1]^T = P  * [x y z 1]^T
    [u-d v 1]^T = P' * [x y z 1]^T

    Combining the two equations above results in the following equation

    [u v u-d 1]^T = [ Fx   0    Cx   0    ] * [ x y z 1]^T
                    [ 0    Fy   Cy   0    ]
                    [ Fx   0    Cx'  FxTx ]
                    [ 0    0    1    0    ]

    Subtracting the 3rd from from the first and inverting the expression
    results in the following equation.

    [x y z 1]^T = Q * [u v d 1]^T

    Where Q is defined as

    Q = [ FyTx  0     0   -FyCxTx     ]
        [ 0     FxTx  0   -FxCyTx     ]
        [ 0     0     0    FxFyTx     ]
        [ 0     0     -Fy  Fy(Cx-Cx') ]

   Using the assumption Fx = Fy Q can be simplified to the following. But for
   compatibility with stereo cameras with different focal lengths we will use
   the full Q matrix.

        [ 1 0   0      -Cx      ]
    Q = [ 0 1   0      -Cy      ]
        [ 0 0   0       Fx      ]
        [ 0 0 -1/Tx (Cx-Cx')/Tx ]

    Disparity = x_left - x_right

    For compatibility with stereo cameras with different focal lengths we will use
    the full Q matrix.

   */
  double Tx = -baseline(); // The baseline member negates our Tx. Undo this negation
  Q_(0,0) =  left_.fy() * Tx;
  Q_(0,3) = -left_.fy() * left_.cx() * Tx;
  Q_(1,1) =  left_.fx() * Tx;
  Q_(1,3) = -left_.fx() * left_.cy() * Tx;
  Q_(2,3) =  left_.fx() * left_.fy() * Tx;
  Q_(3,2) = -left_.fy();
  Q_(3,3) =  left_.fy() * (left_.cx() - right_.cx()); // zero when disparities are pre-adjusted
}

void StereoCameraModel::projectDisparityTo3d(const cv::Point2d& left_uv_rect, float disparity,
                                             cv::Point3d& xyz) const
{
  assert( initialized() );

  // Do the math inline:
  // [X Y Z W]^T = Q * [u v d 1]^T
  // Point = (X/W, Y/W, Z/W)
  // cv::perspectiveTransform could be used but with more overhead.
  double u = left_uv_rect.x, v = left_uv_rect.y;
  cv::Point3d XYZ( (Q_(0,0) * u) + Q_(0,3), (Q_(1,1) * v) + Q_(1,3), Q_(2,3));
  double W = Q_(3,2)*disparity + Q_(3,3);
  xyz = XYZ * (1.0/W);
}

const double StereoCameraModel::MISSING_Z = 10000.;

void StereoCameraModel::projectDisparityImageTo3d(const cv::Mat& disparity, cv::Mat& point_cloud,
                                                  bool handleMissingValues) const
{
  assert( initialized() );

  cv::reprojectImageTo3D(disparity, point_cloud, Q_, handleMissingValues);
}

} //namespace image_geometry
