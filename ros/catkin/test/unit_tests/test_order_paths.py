import os
import unittest
import tempfile
import shutil
from mock import Mock

import imp
imp.load_source('order_paths',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'cmake', 'order_paths.py'))

from order_paths import order_paths, main

class OrderPathsTest(unittest.TestCase):

    def test_order_paths(self):
        self.assertEqual([], order_paths([], []))
        self.assertEqual(['foo/1', 'foo/2', 'bar/1', 'bar/2', 'foo3'],
                         order_paths(['foo3', 'bar/1', 'foo/1', 'foo/2', 'bar/2'], ['foo', 'bar']))
        self.assertEqual(['foo/1', 'foo/2'],
                         order_paths(['foo/1', 'foo/2'], []))
        self.assertEqual(['foo/1', 'foo/2'],
                         order_paths(['foo/1', 'foo/2'], ['']))
