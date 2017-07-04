#!/usr/bin/python

import unittest
import rospy
import PyKDL
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped, Vector3Stamped, PoseStamped

class GeometryMsgs(unittest.TestCase):
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

        v = PointStamped()
        v.header.stamp = rospy.Time(2)
        v.header.frame_id = 'a'
        v.point.x = 1
        v.point.y = 2
        v.point.z = 3
        out = b.transform(v, 'b')
        self.assertEqual(out.point.x, 0)
        self.assertEqual(out.point.y, -2)
        self.assertEqual(out.point.z, -3)

        v = PoseStamped()
        v.header.stamp = rospy.Time(2)
        v.header.frame_id = 'a'
        v.pose.position.x = 1
        v.pose.position.y = 2
        v.pose.position.z = 3
        v.pose.orientation.x = 1
        out = b.transform(v, 'b')
        self.assertEqual(out.pose.position.x, 0)
        self.assertEqual(out.pose.position.y, -2)
        self.assertEqual(out.pose.position.z, -3)

        # Translation shouldn't affect Vector3
        t = TransformStamped()
        t.transform.translation.x = 1
        t.transform.translation.y = 2
        t.transform.translation.z = 3
        t.transform.rotation.w = 1
        v = Vector3Stamped()
        v.vector.x = 1
        v.vector.y = 0
        v.vector.z = 0
        out = tf2_geometry_msgs.do_transform_vector3(v, t)
        self.assertEqual(out.vector.x, 1)
        self.assertEqual(out.vector.y, 0)
        self.assertEqual(out.vector.z, 0)

        # Rotate Vector3 180 deg about y
        t = TransformStamped()
        t.transform.translation.x = 1
        t.transform.translation.y = 2
        t.transform.translation.z = 3
        t.transform.rotation.y = 1

        v = Vector3Stamped()
        v.vector.x = 1
        v.vector.y = 0
        v.vector.z = 0

        out = tf2_geometry_msgs.do_transform_vector3(v, t)
        self.assertEqual(out.vector.x, -1)
        self.assertEqual(out.vector.y, 0)
        self.assertEqual(out.vector.z, 0)

if __name__ == '__main__':
    import rosunit
    rospy.init_node('test_tf2_geometry_msgs_python')
    rosunit.unitrun("test_tf2_geometry_msgs", "test_tf2_geometry_msgs_python", GeometryMsgs)
