import os
import stat
import unittest
import tempfile
import shutil
from mock import Mock

try:
    import catkin.environment_cache
    from catkin.environment_cache import _append_header, _set_variable, _append_comment, _is_not_windows, generate_environment_script
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))


class PlatformTest(unittest.TestCase):

    def setUp(self):
        self.platform_backup = catkin.environment_cache.platform
        self.winplatform = Mock()
        self.winplatform.system.return_value = 'Windows'
        linuxplatform = Mock()
        linuxplatform.system.return_value = 'Linux'
        catkin.environment_cache.platform = linuxplatform

    def tearDown(self):
        catkin.environment_cache.platform = self.platform_backup

    def test_is_not_windows(self):
        self.assertTrue(_is_not_windows())
        catkin.environment_cache.platform = self.winplatform
        self.assertFalse(_is_not_windows())

    def test_appends(self):
        code = []
        _append_header(code)
        self.assertEqual(['#!/usr/bin/env sh',
                          '# generated from catkin/python/catkin/environment_cache.py', ''], code)
        code = []
        _append_comment(code, 'foo')
        self.assertEqual(['# foo'], code)
        code = []
        _set_variable(code, 'foo', 'bar')
        self.assertEqual(['export foo="bar"'], code)

    def test_appends_windows(self):
        catkin.environment_cache.platform = self.winplatform
        code = []
        _append_header(code)
        self.assertEqual(['@echo off',
                          'REM generated from catkin/python/catkin/environment_cache.py',
                          ''], code)
        code = []
        _append_comment(code, 'foo')
        self.assertEqual(['REM foo'], code)
        code = []
        _set_variable(code, 'foo', 'bar')
        self.assertEqual(['set foo=bar'], code)

    def test_generate_environment_script(self):
        try:
            fake_environ = os.environ.copy()
            fake_environ['FOO'] = '/bar'
            fake_environ['TRICK'] = '/lib'
            catkin.environment_cache.os.environ = fake_environ
            rootdir = tempfile.mkdtemp()
            env_file = os.path.join(rootdir, 'env.sh')
            with open(env_file, 'a') as fhand:
                fhand.write('''\
#! /usr/bin/env sh
export FOO=/foo:/bar
export TRICK=/usr/lib
export BAR=/bar
exec "$@"''')
            mode = os.stat(env_file).st_mode
            os.chmod(env_file, mode | stat.S_IXUSR)
            result = generate_environment_script(env_file)
            self.assertTrue('export FOO="/foo:$FOO"' in result, result)
            self.assertTrue('export TRICK="/usr/lib"' in result, result)
            self.assertTrue('export BAR="/bar"' in result, result)
            self.assertEqual('#!/usr/bin/env sh', result[0])
        finally:
            catkin.environment_cache.os.environ = os.environ
            shutil.rmtree(rootdir)
