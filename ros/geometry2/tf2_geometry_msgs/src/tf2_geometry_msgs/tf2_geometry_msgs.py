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

# author: Wim Meeussen

from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped
import PyKDL
import rospy
import tf2_ros

def to_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_to_msg(Vector3Stamped, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(PoseStamped, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(PointStamped, to_msg_msg)

def from_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_from_msg(Vector3Stamped, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(PoseStamped, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(PointStamped, from_msg_msg)

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))


# PointStamped
def do_transform_point(point, transform):
    p = transform_to_kdl(transform) * PyKDL.Vector(point.point.x, point.point.y, point.point.z)
    res = PointStamped()
    res.point.x = p[0]
    res.point.y = p[1]
    res.point.z = p[2]
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(PointStamped, do_transform_point)


# Vector3Stamped
def do_transform_vector3(vector3, transform):
    transform.transform.translation.x = 0;
    transform.transform.translation.y = 0;
    transform.transform.translation.z = 0;
    p = transform_to_kdl(transform) * PyKDL.Vector(vector3.vector.x, vector3.vector.y, vector3.vector.z)
    res = Vector3Stamped()
    res.vector.x = p[0]
    res.vector.y = p[1]
    res.vector.z = p[2]
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(Vector3Stamped, do_transform_vector3)

# PoseStamped
def do_transform_pose(pose, transform):
    f = transform_to_kdl(transform) * PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                                                          pose.pose.orientation.z, pose.pose.orientation.w),
                                                PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
    res = PoseStamped()
    res.pose.position.x = f.p[0]
    res.pose.position.y = f.p[1]
    res.pose.position.z = f.p[2]
    (res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w) = f.M.GetQuaternion()
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(PoseStamped, do_transform_pose)
