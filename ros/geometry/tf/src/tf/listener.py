# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import tf as TFX
from tf import transformations
import numpy

from tf.msg import tfMessage
import rosgraph.masterapi
import geometry_msgs.msg
import sensor_msgs.msg
from tf.srv import FrameGraph,FrameGraphResponse

import threading

def xyz_to_mat44(pos):
    return transformations.translation_matrix((pos.x, pos.y, pos.z))

def xyzw_to_mat44(ori):
    return transformations.quaternion_matrix((ori.x, ori.y, ori.z, ori.w))

## Extends tf's Transformer, adding transform methods for ROS message
## types PointStamped, QuaternionStamped and PoseStamped.
class TransformerROS(TFX.Transformer):
    """
    TransformerROS extends the base class :class:`tf.Transformer`,
    adding methods for handling ROS messages. 
    """

    ## Looks up the transform for ROS message header hdr to frame
    ## target_frame, and returns the transform as a Numpy 4x4 matrix.
    # @param target_frame The target frame
    # @param hdr          A ROS message header object

    def asMatrix(self, target_frame, hdr):
        """
        :param target_frame: the tf target frame, a string
        :param hdr: a message header
        :return: a :class:`numpy.matrix` 4x4 representation of the transform
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise
        
        Uses :meth:`lookupTransform` to look up the transform for ROS message header hdr to frame
        target_frame, and returns the transform as a :class:`numpy.matrix`
        4x4.
        """
        translation,rotation = self.lookupTransform(target_frame, hdr.frame_id, hdr.stamp)
        return self.fromTranslationRotation(translation, rotation)

    ## Returns a Numpy 4x4 matrix for a transform.
    # @param translation  translation as (x,y,z)
    # @param rotation     rotation as (x,y,z,w)

    def fromTranslationRotation(self, translation, rotation):
        """
        :param translation: translation expressed as a tuple (x,y,z)
        :param rotation: rotation quaternion expressed as a tuple (x,y,z,w)
        :return: a :class:`numpy.matrix` 4x4 representation of the transform
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise
        
        Converts a transformation from :class:`tf.Transformer` into a representation as a 4x4 matrix.
        """

        return numpy.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

    ## Transforms a geometry_msgs PointStamped message to frame target_frame, returns the resulting PointStamped.
    # @param target_frame The target frame
    # @param ps           geometry_msgs.msg.PointStamped object

    def transformPoint(self, target_frame, ps):
        """
        :param target_frame: the tf target frame, a string
        :param ps: the geometry_msgs.msg.PointStamped message
        :return: new geometry_msgs.msg.PointStamped message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        Transforms a geometry_msgs PointStamped message to frame target_frame, returns a new PointStamped message.
        """

        mat44 = self.asMatrix(target_frame, ps.header)
        xyz = tuple(numpy.dot(mat44, numpy.array([ps.point.x, ps.point.y, ps.point.z, 1.0])))[:3]
        r = geometry_msgs.msg.PointStamped()
        r.header.stamp = ps.header.stamp
        r.header.frame_id = target_frame
        r.point = geometry_msgs.msg.Point(*xyz)
        return r

    ## Transforms a geometry_msgs Vector3Stamped message to frame target_frame, returns the resulting Vector3Stamped.
    # @param target_frame The target frame
    # @param ps           geometry_msgs.msg.Vector3Stamped object

    def transformVector3(self, target_frame, v3s):
        """
        :param target_frame: the tf target frame, a string
        :param v3s: the geometry_msgs.msg.Vector3Stamped message
        :return: new geometry_msgs.msg.Vector3Stamped message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        Transforms a geometry_msgs Vector3Stamped message to frame target_frame, returns a new Vector3Stamped message.
        """

        mat44 = self.asMatrix(target_frame, v3s.header)
        mat44[0,3] = 0.0
        mat44[1,3] = 0.0
        mat44[2,3] = 0.0
        xyz = tuple(numpy.dot(mat44, numpy.array([v3s.vector.x, v3s.vector.y, v3s.vector.z, 1.0])))[:3]
        r = geometry_msgs.msg.Vector3Stamped()
        r.header.stamp = v3s.header.stamp
        r.header.frame_id = target_frame
        r.vector = geometry_msgs.msg.Vector3(*xyz)
        return r

    ## Transforms a geometry_msgs QuaternionStamped message to frame target_frame, returns the resulting QuaternionStamped.
    # @param target_frame The target frame
    # @param ps           geometry_msgs.msg.QuaternionStamped object

    def transformQuaternion(self, target_frame, ps):
        """
        :param target_frame: the tf target frame, a string
        :param ps: the geometry_msgs.msg.QuaternionStamped message
        :return: new geometry_msgs.msg.QuaternionStamped message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        Transforms a geometry_msgs QuaternionStamped message to frame target_frame, returns a new QuaternionStamped message.
        """

        # mat44 is frame-to-frame transform as a 4x4
        mat44 = self.asMatrix(target_frame, ps.header)

        # pose44 is the given quat as a 4x4
        pose44 = xyzw_to_mat44(ps.quaternion)

        # txpose is the new pose in target_frame as a 4x4
        txpose = numpy.dot(mat44, pose44)

        # quat is orientation of txpose
        quat = tuple(transformations.quaternion_from_matrix(txpose))

        # assemble return value QuaternionStamped
        r = geometry_msgs.msg.QuaternionStamped()
        r.header.stamp = ps.header.stamp
        r.header.frame_id = target_frame
        r.quaternion = geometry_msgs.msg.Quaternion(*quat)
        return r

    ## Transforms a geometry_msgs PoseStamped message to frame target_frame, returns the resulting PoseStamped.
    # @param target_frame The target frame
    # @param ps           geometry_msgs.msg.PoseStamped object

    def transformPose(self, target_frame, ps):
        """
        :param target_frame: the tf target frame, a string
        :param ps: the geometry_msgs.msg.PoseStamped message
        :return: new geometry_msgs.msg.PoseStamped message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        Transforms a geometry_msgs PoseStamped message to frame target_frame, returns a new PoseStamped message.
        """
        # mat44 is frame-to-frame transform as a 4x4
        mat44 = self.asMatrix(target_frame, ps.header)

        # pose44 is the given pose as a 4x4
        pose44 = numpy.dot(xyz_to_mat44(ps.pose.position), xyzw_to_mat44(ps.pose.orientation))

        # txpose is the new pose in target_frame as a 4x4
        txpose = numpy.dot(mat44, pose44)

        # xyz and quat are txpose's position and orientation
        xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
        quat = tuple(transformations.quaternion_from_matrix(txpose))

        # assemble return value PoseStamped
        r = geometry_msgs.msg.PoseStamped()
        r.header.stamp = ps.header.stamp
        r.header.frame_id = target_frame
        r.pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(*xyz), geometry_msgs.msg.Quaternion(*quat))
        return r

    def transformPointCloud(self, target_frame, point_cloud):
        """
        :param target_frame: the tf target frame, a string
        :param ps: the sensor_msgs.msg.PointCloud message
        :return: new sensor_msgs.msg.PointCloud message, in frame target_frame
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

        Transforms a geometry_msgs PoseStamped message to frame target_frame, returns a new PoseStamped message.
        """
        r = sensor_msgs.msg.PointCloud()
        r.header.stamp = point_cloud.header.stamp
        r.header.frame_id = target_frame
        r.channels = point_cloud.channels

        mat44 = self.asMatrix(target_frame, point_cloud.header)
        def xf(p):
            xyz = tuple(numpy.dot(mat44, numpy.array([p.x, p.y, p.z, 1.0])))[:3]
            return geometry_msgs.msg.Point(*xyz)
        r.points = [xf(p) for p in point_cloud.points]
        return r

## Extends TransformerROS, subscribes to the /tf topic and
## updates the Transformer with the messages.

class TransformListenerThread(threading.Thread):
    def __init__(self, tl):
        threading.Thread.__init__(self)
        self.tl = tl
    
    def run(self):
        self.last_update_ros_time = rospy.Time.now()
        rospy.Subscriber("/tf",         tfMessage, self.transformlistener_callback)
        #Check to see if the service has already been advertised in this node
        try:
            m = rosgraph.masterapi.Master(rospy.get_name())
            m.lookupService('~tf_frames')
        except (rosgraph.masterapi.Error, rosgraph.masterapi.Failure):
            self.tl.frame_graph_server = rospy.Service('~tf_frames', FrameGraph, self.frame_graph_service)

        rospy.spin()

    def transformlistener_callback(self, data):
        ros_dt = (rospy.Time.now() - self.last_update_ros_time).to_sec()
        if ros_dt < -0.5:
            rospy.logwarn("Saw a negative time change of %f seconds, clearing the tf buffer." % ros_dt)
            self.tl.clear()
        self.last_update_ros_time = rospy.Time.now()

        who = data._connection_header.get('callerid', "default_authority")
        for transform in data.transforms:
            self.tl.setTransform(transform, who)

    def frame_graph_service(self, req):
        return FrameGraphResponse(self.tl.allFramesAsDot())


class TransformListener(TransformerROS):

    """
    TransformListener is a subclass of :class:`tf.TransformerROS` that
    subscribes to the ``"/tf"`` message topic, and calls :meth:`tf.Transformer.setTransform`
    with each incoming transformation message.

    In this way a TransformListener object automatically
    stays up to to date with all current transforms.  Typical usage might be::

        import tf
        from geometry_msgs.msg import PointStamped

        class MyNode:

            def __init__(self):

                self.tl = tf.TransformListener()
                rospy.Subscriber("/sometopic", PointStamped, self.some_message_handler)
                ...
            
            def some_message_handler(self, point_stamped):

                # want to work on the point in the "world" frame
                point_in_world = self.tl.transformPoint("world", point_stamped)
                ...
        
    """
    def __init__(self, *args):
        TransformerROS.__init__(self, *args)
        thr = TransformListenerThread(self)
        thr.setDaemon(True)
        thr.start()
        self.setUsingDedicatedThread(True)
