import array

import cv2
import sensor_msgs.msg
import math
import copy
import numpy

def mkmat(rows, cols, L):
    mat = numpy.matrix(L, dtype='float64')
    mat.resize((rows,cols))
    return mat

class PinholeCameraModel:

    """
    A pinhole camera is an idealized monocular camera.
    """

    def __init__(self):
        self.K = None
        self.D = None
        self.R = None
        self.P = None
        self.full_K = None
        self.full_P = None
        self.width = None
        self.height = None
        self.binning_x = None
        self.binning_y = None
        self.raw_roi = None
        self.tf_frame = None
        self.stamp = None

    def fromCameraInfo(self, msg):
        """
        :param msg: camera parameters
        :type msg:  sensor_msgs.msg.CameraInfo

        Set the camera parameters from the :class:`sensor_msgs.msg.CameraInfo` message.
        """
        self.K = mkmat(3, 3, msg.K)
        if msg.D:
            self.D = mkmat(len(msg.D), 1, msg.D)
        else:
            self.D = None
        self.R = mkmat(3, 3, msg.R)
        self.P = mkmat(3, 4, msg.P)
        self.full_K = mkmat(3, 3, msg.K)
        self.full_P = mkmat(3, 4, msg.P)
        self.width = msg.width
        self.height = msg.height
        self.binning_x = max(1, msg.binning_x)
        self.binning_y = max(1, msg.binning_y)
        self.resolution = (msg.width, msg.height)

        self.raw_roi = copy.copy(msg.roi)
        # ROI all zeros is considered the same as full resolution
        if (self.raw_roi.x_offset == 0 and self.raw_roi.y_offset == 0 and
            self.raw_roi.width == 0 and self.raw_roi.height == 0):
            self.raw_roi.width = self.width
            self.raw_roi.height = self.height
        self.tf_frame = msg.header.frame_id
        self.stamp = msg.header.stamp

        # Adjust K and P for binning and ROI
        self.K[0,0] /= self.binning_x
        self.K[1,1] /= self.binning_y
        self.K[0,2] = (self.K[0,2] - self.raw_roi.x_offset) / self.binning_x
        self.K[1,2] = (self.K[1,2] - self.raw_roi.y_offset) / self.binning_y
        self.P[0,0] /= self.binning_x
        self.P[1,1] /= self.binning_y
        self.P[0,2] = (self.P[0,2] - self.raw_roi.x_offset) / self.binning_x
        self.P[1,2] = (self.P[1,2] - self.raw_roi.y_offset) / self.binning_y

    def rectifyImage(self, raw, rectified):
        """
        :param raw:       input image
        :type raw:        :class:`CvMat` or :class:`IplImage`
        :param rectified: rectified output image
        :type rectified:  :class:`CvMat` or :class:`IplImage`

        Applies the rectification specified by camera parameters :math:`K` and and :math:`D` to image `raw` and writes the resulting image `rectified`.
        """

        self.mapx = numpy.ndarray(shape=(self.height, self.width, 1),
                           dtype='float32')
        self.mapy = numpy.ndarray(shape=(self.height, self.width, 1),
                           dtype='float32')
        cv2.initUndistortRectifyMap(self.K, self.D, self.R, self.P,
                (self.width, self.height), cv2.CV_32FC1, self.mapx, self.mapy)
        cv2.remap(raw, self.mapx, self.mapy, cv2.INTER_CUBIC, rectified)

    def rectifyPoint(self, uv_raw):
        """
        :param uv_raw:    pixel coordinates
        :type uv_raw:     (u, v)

        Applies the rectification specified by camera parameters
        :math:`K` and and :math:`D` to point (u, v) and returns the
        pixel coordinates of the rectified point.
        """

        src = mkmat(1, 2, list(uv_raw))
        src.resize((1,1,2))
        dst = cv2.undistortPoints(src, self.K, self.D, R=self.R, P=self.P)
        return dst[0,0]

    def project3dToPixel(self, point):
        """
        :param point:     3D point
        :type point:      (x, y, z)

        Returns the rectified pixel coordinates (u, v) of the 3D point,
        using the camera :math:`P` matrix.
        This is the inverse of :meth:`projectPixelTo3dRay`.
        """
        src = mkmat(4, 1, [point[0], point[1], point[2], 1.0])
        dst = self.P * src
        x = dst[0,0]
        y = dst[1,0]
        w = dst[2,0]
        if w != 0:
            return (x / w, y / w)
        else:
            return (float('nan'), float('nan'))

    def projectPixelTo3dRay(self, uv):
        """
        :param uv:        rectified pixel coordinates
        :type uv:         (u, v)

        Returns the unit vector which passes from the camera center to through rectified pixel (u, v),
        using the camera :math:`P` matrix.
        This is the inverse of :meth:`project3dToPixel`.
        """
        x = (uv[0] - self.cx()) / self.fx()
        y = (uv[1] - self.cy()) / self.fy()
        norm = math.sqrt(x*x + y*y + 1)
        x /= norm
        y /= norm
        z = 1.0 / norm
        return (x, y, z)

    def getDeltaU(self, deltaX, Z):
        """
        :param deltaX:          delta X, in cartesian space
        :type deltaX:           float
        :param Z:               Z, in cartesian space
        :type Z:                float
        :rtype:                 float

        Compute delta u, given Z and delta X in Cartesian space.
        For given Z, this is the inverse of :meth:`getDeltaX`.
        """
        fx = self.P[0, 0]
        if Z == 0:
            return float('inf')
        else:
            return fx * deltaX / Z

    def getDeltaV(self, deltaY, Z):
        """
        :param deltaY:          delta Y, in cartesian space
        :type deltaY:           float
        :param Z:               Z, in cartesian space
        :type Z:                float
        :rtype:                 float

        Compute delta v, given Z and delta Y in Cartesian space.
        For given Z, this is the inverse of :meth:`getDeltaY`.
        """
        fy = self.P[1, 1]
        if Z == 0:
            return float('inf')
        else:
            return fy * deltaY / Z

    def getDeltaX(self, deltaU, Z):
        """
        :param deltaU:          delta u in pixels
        :type deltaU:           float
        :param Z:               Z, in cartesian space
        :type Z:                float
        :rtype:                 float

        Compute delta X, given Z in cartesian space and delta u in pixels.
        For given Z, this is the inverse of :meth:`getDeltaU`.
        """
        fx = self.P[0, 0]
        return Z * deltaU / fx

    def getDeltaY(self, deltaV, Z):
        """
        :param deltaV:          delta v in pixels
        :type deltaV:           float
        :param Z:               Z, in cartesian space
        :type Z:                float
        :rtype:                 float

        Compute delta Y, given Z in cartesian space and delta v in pixels.
        For given Z, this is the inverse of :meth:`getDeltaV`.
        """
        fy = self.P[1, 1]
        return Z * deltaV / fy

    def fullResolution(self):
        """Returns the full resolution of the camera"""
        return self.resolution

    def intrinsicMatrix(self):
        """ Returns :math:`K`, also called camera_matrix in cv docs """
        return self.K
    def distortionCoeffs(self):
        """ Returns :math:`D` """
        return self.D
    def rotationMatrix(self):
        """ Returns :math:`R` """
        return self.R
    def projectionMatrix(self):
        """ Returns :math:`P` """
        return self.P
    def fullIntrinsicMatrix(self):
        """ Return the original camera matrix for full resolution """
        return self.full_K
    def fullProjectionMatrix(self):
        """ Return the projection matrix for full resolution """
        return self.full_P

    def cx(self):
        """ Returns x center """
        return self.P[0,2]
    def cy(self):
        """ Returns y center """
        return self.P[1,2]
    def fx(self):
        """ Returns x focal length """
        return self.P[0,0]
    def fy(self):
        """ Returns y focal length """
        return self.P[1,1]

    def Tx(self):
        """ Return the x-translation term of the projection matrix """
        return self.P[0,3]

    def Ty(self):
        """ Return the y-translation term of the projection matrix """
        return self.P[1,3]

    def tfFrame(self):
        """ Returns the tf frame name - a string - of the camera.
        This is the frame of the :class:`sensor_msgs.msg.CameraInfo` message.
        """
        return self.tf_frame

class StereoCameraModel:
    """
    An idealized stereo camera.
    """
    def __init__(self):
        self.left = PinholeCameraModel()
        self.right = PinholeCameraModel()

    def fromCameraInfo(self, left_msg, right_msg):
        """
        :param left_msg: left camera parameters
        :type left_msg:  sensor_msgs.msg.CameraInfo
        :param right_msg: right camera parameters
        :type right_msg:  sensor_msgs.msg.CameraInfo

        Set the camera parameters from the :class:`sensor_msgs.msg.CameraInfo` messages.
        """
        self.left.fromCameraInfo(left_msg)
        self.right.fromCameraInfo(right_msg)

        # [ Fx, 0,  Cx,  Fx*-Tx ]
        # [ 0,  Fy, Cy,  0      ]
        # [ 0,  0,  1,   0      ]

        fx = self.right.P[0, 0]
        fy = self.right.P[1, 1]
        cx = self.right.P[0, 2]
        cy = self.right.P[1, 2]
        tx = -self.right.P[0, 3] / fx

        # Q is:
        #    [ 1, 0,  0, -Clx ]
        #    [ 0, 1,  0, -Cy ]
        #    [ 0, 0,  0,  Fx ]
        #    [ 0, 0, 1 / Tx, (Crx-Clx)/Tx ]

        self.Q = numpy.zeros((4, 4), dtype='float64')
        self.Q[0, 0] = 1.0
        self.Q[0, 3] = -cx
        self.Q[1, 1] = 1.0
        self.Q[1, 3] = -cy
        self.Q[2, 3] = fx
        self.Q[3, 2] = 1 / tx

    def tfFrame(self):
        """
        Returns the tf frame name - a string - of the 3d points.  This is
        the frame of the :class:`sensor_msgs.msg.CameraInfo` message.  It
        may be used as a source frame in :class:`tf.TransformListener`.
        """

        return self.left.tfFrame()

    def project3dToPixel(self, point):
        """
        :param point:     3D point
        :type point:      (x, y, z)

        Returns the rectified pixel coordinates (u, v) of the 3D point, for each camera, as ((u_left, v_left), (u_right, v_right))
        using the cameras' :math:`P` matrices.
        This is the inverse of :meth:`projectPixelTo3d`.
        """
        l = self.left.project3dToPixel(point)
        r = self.right.project3dToPixel(point)
        return (l, r)

    def projectPixelTo3d(self, left_uv, disparity):
        """
        :param left_uv:        rectified pixel coordinates
        :type left_uv:         (u, v)
        :param disparity:        disparity, in pixels
        :type disparity:         float

        Returns the 3D point (x, y, z) for the given pixel position,
        using the cameras' :math:`P` matrices.
        This is the inverse of :meth:`project3dToPixel`.

        Note that a disparity of zero implies that the 3D point is at infinity.
        """
        src = mkmat(4, 1, [left_uv[0], left_uv[1], disparity, 1.0])
        dst = self.Q * src
        x = dst[0, 0]
        y = dst[1, 0]
        z = dst[2, 0]
        w = dst[3, 0]
        if w != 0:
            return (x / w, y / w, z / w)
        else:
            return (0.0, 0.0, 0.0)

    def getZ(self, disparity):
        """
        :param disparity:        disparity, in pixels
        :type disparity:         float

        Returns the depth at which a point is observed with a given disparity.
        This is the inverse of :meth:`getDisparity`.

        Note that a disparity of zero implies Z is infinite.
        """
        if disparity == 0:
            return float('inf')
        Tx = -self.right.P[0, 3]
        return Tx / disparity

    def getDisparity(self, Z):
        """
        :param Z:          Z (depth), in cartesian space
        :type Z:           float

        Returns the disparity observed for a point at depth Z.
        This is the inverse of :meth:`getZ`.
        """
        if Z == 0:
            return float('inf')
        Tx = -self.right.P[0, 3]
        return Tx / Z
