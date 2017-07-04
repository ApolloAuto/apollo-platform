import em
import os
import sys
import stat
import unittest
import tempfile
import shutil

class OrderPackagesEmTest(unittest.TestCase):

    def test_env_cached_static(self):
        # hack to fix empy nosetests clash
        sys.stdout = em.ProxyFile(sys.stdout)
        template_file = os.path.join(os.path.dirname(__file__), '..', '..', 'cmake', 'em', 'order_packages.cmake.em')
        with open (template_file, 'r') as fhand:
            template = fhand.read()
        gdict = {'CATKIN_DEVEL_PREFIX': '/foo',
                 'CMAKE_PREFIX_PATH': ['/bar'],
                 'CATKIN_GLOBAL_LIB_DESTINATION': '/glob-dest/lib',
                 'CATKIN_GLOBAL_BIN_DESTINATION': '/glob-dest/bin',
                 'PYTHON_INSTALL_DIR': '/foo/dist-packages'}
        result = em.expand(template, gdict,
                           source_root_dir='/tmp/nowhere_dir',
                           whitelisted_packages=[],
                           blacklisted_packages=[],
                           underlay_workspaces=[])
        self.assertTrue('set(CATKIN_ORDERED_PACKAGES "")' in result, result)
        self.assertTrue('set(CATKIN_ORDERED_PACKAGE_PATHS "")' in result, result)
        self.assertTrue('set(CATKIN_ORDERED_PACKAGES_IS_META "")' in result, result)
        self.assertTrue('set(CATKIN_ORDERED_PACKAGES_BUILD_TYPE "")' in result, result)
        self.assertTrue('set(CATKIN_MESSAGE_GENERATORS' in result, result)
        self.assertEqual(10, len(result.splitlines()))
