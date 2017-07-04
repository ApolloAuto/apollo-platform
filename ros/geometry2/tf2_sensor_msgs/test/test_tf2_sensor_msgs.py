#!/usr/bin/env python

import unittest
import struct
import tf2_sensor_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from tf2_ros import TransformStamped
import copy

## A sample python unit test
class PointCloudConversions(unittest.TestCase):
    def setUp(self):
        self.point_cloud_in = point_cloud2.PointCloud2()
        self.point_cloud_in.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                                PointField('y', 4, PointField.FLOAT32, 1),
                                PointField('z', 8, PointField.FLOAT32, 1)]

        self.point_cloud_in.point_step = 4 * 3
        self.point_cloud_in.height = 1
        # we add two points (with x, y, z to the cloud)
        self.point_cloud_in.width = 2
        self.point_cloud_in.row_step = self.point_cloud_in.point_step * self.point_cloud_in.width

        points = [1, 2, 0, 10, 20, 30]
        self.point_cloud_in.data = struct.pack('%sf' % len(points), *points)


        self.transform_translate_xyz_300 = TransformStamped()
        self.transform_translate_xyz_300.transform.translation.x = 300
        self.transform_translate_xyz_300.transform.translation.y = 300
        self.transform_translate_xyz_300.transform.translation.z = 300
        self.transform_translate_xyz_300.transform.rotation.w = 1  # no rotation so we only set w

        assert(list(point_cloud2.read_points(self.point_cloud_in)) == [(1.0, 2.0, 0.0), (10.0, 20.0, 30.0)])

    def test_simple_transform(self):
        old_data = copy.deepcopy(self.point_cloud_in.data)  # deepcopy is not required here because we have a str for now
        point_cloud_transformed = tf2_sensor_msgs.do_transform_cloud(self.point_cloud_in, self.transform_translate_xyz_300)

        k = 300
        expected_coordinates = [(1+k, 2+k, 0+k), (10+k, 20+k, 30+k)]
        new_points = list(point_cloud2.read_points(point_cloud_transformed))
        print("new_points are %s" % new_points)
        assert(expected_coordinates == new_points)
        assert(old_data == self.point_cloud_in.data)  # checking no modification in input cloud

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("test_tf2_sensor_msgs", "test_point_cloud_conversion", PointCloudConversions)

