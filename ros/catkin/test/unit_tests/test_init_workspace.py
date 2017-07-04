import os
from os.path import join
import unittest
import tempfile
import shutil

try:
    from catkin.init_workspace import init_workspace, _symlink_or_copy
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))


class InitWorkspaceTest(unittest.TestCase):

    def test_symlink_or_copy(self):
        try:
            root_dir = tempfile.mkdtemp()
            os.makedirs(join(root_dir, 'subdir'))
            os.makedirs(join(root_dir, 'subdir2'))
            with open(join(root_dir, 'subdir', 'foo'), 'ab') as fhand:
                fhand.write('content'.encode('UTF-8'))

            _symlink_or_copy(join(root_dir, 'subdir', 'foo'),
                             join(root_dir, 'foolink'))
            _symlink_or_copy(join(root_dir, 'subdir', 'foo'),
                             join(root_dir, 'subdir', 'foolink'))
            _symlink_or_copy(os.path.relpath(join(root_dir, 'subdir', 'foo'),
                                             os.getcwd()),
                             join(root_dir, 'foolinkrel'))

            self.assertEqual(join(root_dir, 'subdir', 'foo'),
                             os.readlink(join(root_dir, 'foolink')))
            self.assertEqual(join(root_dir, 'subdir', 'foo'),
                             os.readlink(join(root_dir, 'subdir', 'foolink')))
            self.assertEqual(os.path.relpath(join(root_dir, 'subdir', 'foo'),
                                             os.getcwd()),
                             os.readlink(join(root_dir, 'foolinkrel')))

        finally:
            # pass
            shutil.rmtree(root_dir)

    def test_init_workspace(self):
        try:
            root_dir = tempfile.mkdtemp()
            os.makedirs(join(root_dir, 'ws1'))
            os.makedirs(join(root_dir, 'ws1', 'catkin'))
            os.makedirs(join(root_dir, 'ws1', 'catkin', 'cmake'))
            with open(join(root_dir, 'ws1', 'catkin', 'cmake', 'toplevel.cmake'),
                      'ab') as fhand:
                fhand.write(''.encode('UTF-8'))
            with open(join(root_dir, 'ws1', '.catkin'), 'ab') as fhand:
                fhand.write(''.encode('UTF-8'))
            os.makedirs(join(root_dir, 'ws2'))
            with open(join(root_dir, 'ws2', '.catkin'), 'ab') as fhand:
                fhand.write(''.encode('UTF-8'))

            init_workspace(join(root_dir, 'ws1'))
            init_workspace(join(root_dir, 'ws2'))

            # in same workspace symlink should be relative
            self.assertEqual(
                join('catkin', 'cmake', 'toplevel.cmake'),
                os.readlink(join(root_dir, 'ws1', 'CMakeLists.txt')))
            # outside workspace, path should be absolute
            self.assertTrue(
                os.path.samefile(
                    join(os.path.dirname(__file__),
                         '..', '..', 'cmake', 'toplevel.cmake'),
                    os.readlink(join(root_dir, 'ws2', 'CMakeLists.txt'))))

        finally:
            # pass
            shutil.rmtree(root_dir)
