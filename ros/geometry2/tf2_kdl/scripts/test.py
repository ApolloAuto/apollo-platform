#!/usr/bin/python

import unittest
import rospy
import PyKDL
import tf2_ros
import tf2_kdl
from geometry_msgs.msg import TransformStamped
from copy import deepcopy

class KDLConversions(unittest.TestCase):
    def test_transform(self):
        b = tf2_ros.Buffer()
        t = TransformStamped()
        t.transform.translation.x = 1
        t.transform.rotation.x = 1
        t.header.stamp = rospy.Time(2.0)
        t.header.frame_id = 'a'
        t.child_frame_id = 'b'
        b.set_transform(t, 'eitan_rocks')
        out = b.lookup_transform('a','b', rospy.Time(2.0), rospy.Duration(2.0))
        self.assertEqual(out.transform.translation.x, 1)
        self.assertEqual(out.transform.rotation.x, 1)
        self.assertEqual(out.header.frame_id, 'a')
        self.assertEqual(out.child_frame_id, 'b')

        v = PyKDL.Vector(1,2,3)
        out = b.transform(tf2_ros.Stamped(v, rospy.Time(2), 'a'), 'b')
        self.assertEqual(out.x(), 0)
        self.assertEqual(out.y(), -2)
        self.assertEqual(out.z(), -3)

        f = PyKDL.Frame(PyKDL.Rotation.RPY(1,2,3), PyKDL.Vector(1,2,3))
        out = b.transform(tf2_ros.Stamped(f, rospy.Time(2), 'a'), 'b')
        print out
        self.assertEqual(out.p.x(), 0)
        self.assertEqual(out.p.y(), -2)
        self.assertEqual(out.p.z(), -3)
        # TODO(tfoote) check values of rotation

        t = PyKDL.Twist(PyKDL.Vector(1,2,3), PyKDL.Vector(4,5,6))
        out = b.transform(tf2_ros.Stamped(t, rospy.Time(2), 'a'), 'b')
        self.assertEqual(out.vel.x(), 1)
        self.assertEqual(out.vel.y(), -8)
        self.assertEqual(out.vel.z(), 2)
        self.assertEqual(out.rot.x(), 4)
        self.assertEqual(out.rot.y(), -5)
        self.assertEqual(out.rot.z(), -6)

        w = PyKDL.Wrench(PyKDL.Vector(1,2,3), PyKDL.Vector(4,5,6))
        out = b.transform(tf2_ros.Stamped(w, rospy.Time(2), 'a'), 'b')
        self.assertEqual(out.force.x(), 1)
        self.assertEqual(out.force.y(), -2)
        self.assertEqual(out.force.z(), -3)
        self.assertEqual(out.torque.x(), 4)
        self.assertEqual(out.torque.y(), -8)
        self.assertEqual(out.torque.z(), -4)

    def test_convert(self):
        v = PyKDL.Vector(1,2,3)
        vs = tf2_ros.Stamped(v, rospy.Time(2), 'a')
        vs2 = tf2_ros.convert(vs, PyKDL.Vector)
        self.assertEqual(vs.x(), 1)
        self.assertEqual(vs.y(), 2)
        self.assertEqual(vs.z(), 3)
        self.assertEqual(vs2.x(), 1)
        self.assertEqual(vs2.y(), 2)
        self.assertEqual(vs2.z(), 3)


if __name__ == '__main__':
    import rosunit
    rospy.init_node('test_tf2_kdl_python')
    rosunit.unitrun("test_tf2_kdl", "test_tf2_kdl_python", KDLConversions)
