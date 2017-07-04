import os
import unittest
import tempfile
import shutil

try:
    from catkin.package_version import _replace_version, update_versions
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))


class PackageVersionTest(unittest.TestCase):

    def test_replace_version(self):
        self.assertEqual("<package><version>0.1.1</version></package>",
                         _replace_version("<package><version>0.1.0</version></package>", "0.1.1"))
        self.assertEqual("<package><version abi='0.1.0'>0.1.1</version></package>",
                         _replace_version("<package><version abi='0.1.0'>0.1.0</version></package>", "0.1.1"))
        self.assertRaises(RuntimeError, _replace_version, "<package></package>", "0.1.1")
        self.assertRaises(RuntimeError, _replace_version, "<package><version>0.1.1</version><version>0.1.1</version></package>", "0.1.1")

    def test_update_versions(self):
        try:
            root_dir = tempfile.mkdtemp()
            sub_dir = os.path.join(root_dir, 'sub')
            with open(os.path.join(root_dir, "package.xml"), 'w') as fhand:
                fhand.write('<package><version>2.3.4</version></package>')
            os.makedirs(os.path.join(sub_dir))
            with open(os.path.join(sub_dir, "package.xml"), 'w') as fhand:
                fhand.write('<package><version>1.5.4</version></package>')

            update_versions([root_dir, sub_dir], "7.6.5")

            with open(os.path.join(root_dir, "package.xml"), 'r') as fhand:
                contents = fhand.read()
                self.assertEqual('<package><version>7.6.5</version></package>', contents)
            with open(os.path.join(sub_dir, "package.xml"), 'r') as fhand:
                contents = fhand.read()
                self.assertEqual('<package><version>7.6.5</version></package>', contents)
        finally:
            shutil.rmtree(root_dir)
