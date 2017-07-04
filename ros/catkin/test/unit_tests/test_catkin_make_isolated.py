from __future__ import print_function

import os
import shutil
import subprocess
import sys
import tempfile
import unittest

try:
    from catkin.builder import extract_cmake_and_make_arguments
except ImportError as e:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(e)
    )

import imp
imp.load_source('catkin_make_isolated',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'bin', 'catkin_make_isolated'))

from catkin_make_isolated import handle_cmake_args
from catkin_make_isolated import main
from catkin_make_isolated import parse_args


class CatkinMakeIsolatedTests(unittest.TestCase):

    def test_extract_cmake_and_make_arguments(self):
        args = []
        args, cmake_args, make_args = extract_cmake_and_make_arguments(args)
        assert cmake_args == []
        assert args == []

        args = ['-DCMAKE_INSTALL_PREFIX=install']
        args, cmake_args, make_args = extract_cmake_and_make_arguments(args)
        assert cmake_args == ['-DCMAKE_INSTALL_PREFIX=install']
        assert args == []

        args = ['-DCMAKE_INSTALL_PREFIX=install', '--install']
        args, cmake_args, make_args = extract_cmake_and_make_arguments(args)
        assert cmake_args == ['-DCMAKE_INSTALL_PREFIX=install']
        assert args == ['--install']

        args = [
            '-DCMAKE_INSTALL_PREFIX=install', '--install', '--install-space',
            'install_isolated'
        ]
        args, cmake_args, make_args = extract_cmake_and_make_arguments(args)
        assert cmake_args == ['-DCMAKE_INSTALL_PREFIX=install']
        assert args == ['--install', '--install-space', 'install_isolated']

        args = ['-DCATKIN_DEVEL_PREFIX=devel']
        args, cmake_args, make_args = extract_cmake_and_make_arguments(args)
        assert cmake_args == ['-DCATKIN_DEVEL_PREFIX=devel']
        assert args == []

        args = ['-DCATKIN_DEVEL_PREFIX=devel']
        args, cmake_args, make_args = extract_cmake_and_make_arguments(args)
        assert cmake_args == ['-DCATKIN_DEVEL_PREFIX=devel']
        assert args == []

        args = [
            '-DCATKIN_DEVEL_PREFIX=devel', '--devel-space',
            'devel_isolated'
        ]
        args, cmake_args, make_args = extract_cmake_and_make_arguments(args)
        assert cmake_args == ['-DCATKIN_DEVEL_PREFIX=devel']
        assert args == ['--devel-space', 'devel_isolated']

    def test_handle_cmake_args(self):
        args = ['-DCMAKE_INSTALL_PREFIX=install', '--install']
        args, cmake_args, make_args = extract_cmake_and_make_arguments(args)
        assert cmake_args == ['-DCMAKE_INSTALL_PREFIX=install'], cmake_args
        opts = parse_args(args)
        cmake_args, opts = handle_cmake_args(cmake_args, opts)
        assert cmake_args == [], cmake_args
        assert opts.install == True
        assert opts.install_space == 'install'

        args = [
            '-DCMAKE_INSTALL_PREFIX=install', '--install', '--install-space',
            'install_isolated'
        ]
        args, cmake_args, make_args = extract_cmake_and_make_arguments(args)
        assert cmake_args == ['-DCMAKE_INSTALL_PREFIX=install'], cmake_args
        opts = parse_args(args)
        cmake_args, opts = handle_cmake_args(cmake_args, opts)
        assert cmake_args == [], cmake_args
        assert opts.install == True
        assert opts.install_space == 'install_isolated'

        args = ['-DCATKIN_DEVEL_PREFIX=devel']
        args, cmake_args, make_args = extract_cmake_and_make_arguments(args)
        assert cmake_args == ['-DCATKIN_DEVEL_PREFIX=devel'], cmake_args
        opts = parse_args(args)
        cmake_args, opts = handle_cmake_args(cmake_args, opts)
        assert cmake_args == [], cmake_args
        assert opts.devel == 'devel'

        args = [
            '-DCATKIN_DEVEL_PREFIX=devel', '--devel-space',
            'devel_isolated'
        ]
        args, cmake_args, make_args = extract_cmake_and_make_arguments(args)
        assert cmake_args == ['-DCATKIN_DEVEL_PREFIX=devel'], cmake_args
        opts = parse_args(args)
        cmake_args, opts = handle_cmake_args(cmake_args, opts)
        assert cmake_args == [], cmake_args
        assert opts.devel == 'devel_isolated'

    def test_empty_workspace(self):
        argv = sys.argv
        environ = os.environ
        error_msg = None
        try:
            ws_dir = tempfile.mkdtemp()
            src_dir = os.path.join(ws_dir, 'src')
            os.mkdir(src_dir)
            sys.argv = ['catkin_make_isolated', '-C', ws_dir]
            environ['CMAKE_PREFIX_PATH'] = os.path.join(ws_dir, 'install')
            main()
        except Exception as e:
            error_msg = str(e)
        finally:
            shutil.rmtree(ws_dir)
            sys.argv = argv
            os.environ = environ
            assert error_msg is None, error_msg

    def test_symlinked_src(self):
        argv = sys.argv
        environ = os.environ
        cwd = os.getcwd()
        error_msg = None
        try:
            base_dir = tempfile.mkdtemp()
            ws_dir = os.path.join(base_dir, 'ws')
            os.mkdir(ws_dir)
            other_dir = os.path.join(base_dir, 'other')
            os.mkdir(other_dir)
            src_dir = os.path.join(ws_dir, 'src')
            os.symlink(other_dir, src_dir)

            cmi = os.path.join(os.path.dirname(__file__), '..', '..', 'bin', 'catkin_make_isolated')
            environ['CMAKE_PREFIX_PATH'] = os.path.join(ws_dir, 'install')
            environ['PWD'] = src_dir
            subprocess.check_output(' '.join([cmi, '-C', '..']), cwd=src_dir, env=environ, shell=True)
        except Exception as e:
            error_msg = str(e)
        finally:
            shutil.rmtree(ws_dir)
            sys.argv = argv
            os.environ = environ
            os.chdir(cwd)
            assert error_msg is None, error_msg
