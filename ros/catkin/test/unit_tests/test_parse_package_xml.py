import os
import unittest
import tempfile
import shutil
from mock import Mock

import imp
imp.load_source('parse_package_xml',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'cmake', 'parse_package_xml.py'))

from parse_package_xml import _get_output, main


class ParsePackageXmlTest(unittest.TestCase):

    def test_get_output(self):
        pack = Mock()
        pack.package_format = 2
        pack.name = 'foopack'
        pack.version = '0.1.2'
        pack.maintainers = ['m1', 'm2']
        pack.build_depends = ['bd1', 'bd2']
        pack.buildtool_depends = ['catkin']
        pack.build_export_depends = ['bed1', 'bed2']
        pack.buildtool_export_depends = ['bted1', 'bted2']
        pack.exec_depends = ['ed1', 'ed2']
        pack.run_depends = ['rd1', 'rd2']
        pack.test_depends = ['td1', 'td2']
        pack.doc_depends = ['dd1', 'dd2']
        pack.exports = []
        result = _get_output(pack)
        self.assertEqual(
            set([
                'set(_CATKIN_CURRENT_PACKAGE "foopack")',
                'set(foopack_MAINTAINER "m1, m2")',
                'set(foopack_PACKAGE_FORMAT "2")',
                'set(foopack_DEPRECATED "")',
                'set(foopack_VERSION "0.1.2")',
                'set(foopack_BUILD_DEPENDS "bd1" "bd2")',
                'set(foopack_BUILDTOOL_DEPENDS "catkin")',
                'set(foopack_BUILD_EXPORT_DEPENDS "bed1" "bed2")',
                'set(foopack_BUILDTOOL_EXPORT_DEPENDS "bted1" "bted2")',
                'set(foopack_EXEC_DEPENDS "ed1" "ed2")',
                'set(foopack_RUN_DEPENDS "rd1" "rd2")',
                'set(foopack_TEST_DEPENDS "td1" "td2")',
                'set(foopack_DOC_DEPENDS "dd1" "dd2")',
            ]),
            set(result))

    def test_main(self):
        try:
            rootdir = tempfile.mkdtemp()
            src_file = os.path.join(rootdir, 'package.xml')
            check_file = os.path.join(rootdir, 'foo.cmake')
            with open(src_file, 'w') as fhand:
                fhand.write('''<package>
<name>foopack</name>
<version>0.1.2</version>
<description>foo</description>
<maintainer email='foo@bar.com'>foo</maintainer>
<license>foo</license>
<run_depend>rd1</run_depend>
<run_depend>rd2</run_depend>
<build_depend>bd1</build_depend>
<build_depend>bd2</build_depend>
</package>''')
            main([src_file, check_file])
            self.assertTrue(os.path.isfile(check_file))
            with open(check_file, 'r') as fhand:
                contents = fhand.read()
            self.assertEqual(
                set([
                    'set(_CATKIN_CURRENT_PACKAGE "foopack")',
                    'set(foopack_MAINTAINER "foo <foo@bar.com>")',
                    'set(foopack_PACKAGE_FORMAT "1")',
                    'set(foopack_DEPRECATED "")',
                    'set(foopack_VERSION "0.1.2")',
                    'set(foopack_BUILD_DEPENDS "bd1" "bd2")',
                    'set(foopack_BUILDTOOL_DEPENDS )',
                    'set(foopack_BUILD_EXPORT_DEPENDS "rd1" "rd2")',
                    'set(foopack_BUILDTOOL_EXPORT_DEPENDS )',
                    'set(foopack_EXEC_DEPENDS "rd1" "rd2")',
                    'set(foopack_RUN_DEPENDS "rd1" "rd2")',
                    'set(foopack_TEST_DEPENDS )',
                    'set(foopack_DOC_DEPENDS )',
                ]),
                set(contents.splitlines()))
        finally:
            shutil.rmtree(rootdir)
