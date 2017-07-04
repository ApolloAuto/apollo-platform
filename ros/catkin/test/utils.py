from __future__ import print_function

from sys import version_info

import os
import shutil

import subprocess
import unittest
import tempfile

# import platform
# ubuntudist = platform.dist()[2]

PYTHON_INSTALL_PATH = os.path.join('lib',
                                   'python%u.%u' % (version_info[0],
                                                    version_info[1]),
                                   'dist-packages')


TESTS_DIR = os.path.dirname(__file__)
CATKIN_DIR = os.path.dirname(TESTS_DIR)

TEMP_DIR = os.path.join(TESTS_DIR, 'tmp')
if not os.path.isdir(TEMP_DIR):
    os.makedirs(TEMP_DIR)

# network_tests_path = os.path.join(TESTS_DIR, 'network_tests')
MOCK_DIR = os.path.join(TESTS_DIR, 'mock_resources')

# MAKE_CMD = ['make', 'VERBOSE=1', '-j8']
MAKE_CMD = ['make', '-j8']


def rosinstall(pth, specfile):
    '''
    calls rosinstall in pth with given specfile,
    then replaces CMakelists with catkin's toplevel.cmake'
    '''
    assert os.path.exists(specfile), specfile
    # to save testing time, we do not invoke rosinstall when we
    # already have a .rosinstall file
    if not os.path.exists(os.path.join(pth, '.rosinstall')):
        succeed(["rosinstall", "-j8", "--catkin", "-n",
                 pth, specfile, '--continue-on-error'], cwd=TESTS_DIR)


def run(args, **kwargs):
    """
    Call to Popen, returns (errcode, stdout, stderr)
    """
    print("run:", args)
    p = subprocess.Popen(args,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT,
                         cwd=kwargs.get('cwd', None))
    print("P==", p.__dict__)
    (stdout, stderr) = p.communicate()
    return (p.returncode, stdout, stderr)


def create_catkin_workspace(pth):
    """
    prepares path to be a catkin workspace, by copying catkin and
    creating a CMakeLists.txt
    """
    if not os.path.isdir(pth):
        os.makedirs(pth)

    catkin_dir = os.path.join(pth, 'catkin')
    if os.path.isdir(catkin_dir):
        shutil.rmtree(catkin_dir)
    # copy current catkin sources into workspace
    # avoid copying tmp, as that may contain all of ros core

    def notest(folder, contents):
        if folder.endswith('test'):
            return ['tmp']
        return []
    shutil.copytree(CATKIN_DIR, catkin_dir, symlinks=True, ignore=notest)
    assert (os.path.exists(pth + "/catkin/cmake/toplevel.cmake")), \
        pth + "/catkin/cmake/toplevel.cmake"
    # workaround for current rosinstall creating flawed CMakelists
    workspace_cmake = os.path.join(pth, "CMakeLists.txt")
    if os.path.isfile(workspace_cmake):
        os.remove(workspace_cmake)
    succeed(["/bin/ln", "-s", "catkin/cmake/toplevel.cmake",
             "CMakeLists.txt"],
            cwd=pth)


def succeed(cmd, **kwargs):
    """
    Call to Popen, returns stdout, or fails
    """
    print(">>>", cmd, kwargs)
    (r, out, err) = run(cmd, **kwargs)
    print("<<<", out)
    assert r == 0, "cmd failed with result %s:\n %s " % (r, str(cmd))
    return out


def fail(cmd, **kwargs):
    """
    runs command expecting it to return non-zero
    """
    print(">>>", cmd, kwargs)
    (r, out, err) = run(cmd, withexitstatus=True, **kwargs)
    print("<<<", out)
    assert 0 != r, """cmd succeeded, though should fail: %s
  result=%u\n  output=\n%s""" % (cmd, r, out)
    return out


class AbstractCatkinWorkspaceTest(unittest.TestCase):

    """
    Parent class for any test case that creates a workspace and calls
    cmake, make, and make install. Creates a suitable folder structure
    either in /tmp or in a root folder specified on init, that is a
    build folder, and a src folder with latest catkin from source.
    """

    def __init__(self, testCaseName, rootdir=None):
        super(AbstractCatkinWorkspaceTest, self).__init__(testCaseName)
        self.rootdir = rootdir

    def setUp(self):
        # directories to delete in teardown
        self.directories = {}
        if self.rootdir is None:
            self.rootdir = tempfile.mkdtemp()
        self.directories['root'] = self.rootdir
        self.builddir = os.path.join(self.rootdir, "build")
        self.develspace = os.path.join(self.builddir, 'devel')
        self.workspacedir = os.path.join(self.rootdir, "src")
        self.installdir = os.path.join(self.rootdir, "install")
        if not os.path.exists(self.builddir):
            os.makedirs(self.builddir)
        self.setupWorkspaceContents()

    def setupWorkspaceContents(self):
        create_catkin_workspace(self.workspacedir)

    # comment this to investigate results, cleanup tmp folders
    # manually
    def tearDown(self):
        for d in self.directories:
            shutil.rmtree(self.directories[d])
        self.directories = {}

    def cmake(self,
              cwd=None,
              srcdir=None,
              installdir=None,
              prefix_path=None,
              expect=succeed,
              **kwargs):
        """
        invokes cmake
        :param cwd: changes build dir
        :param srcdir: changes sourcedir
        :param installdir: changes installdir
        :param prefix_path: where to cmake against (where to find)
        :param expect: one of functions: succeed, fail
        :param kwargs: (cwd, srcdir, expect) or stuff that will be
        added to the cmake command
        """
        args = []
        if cwd is None:
            cwd = self.builddir
        if srcdir is None:
            srcdir = self.workspacedir
        this_builddir = cwd
        this_srcdir = srcdir
        print("v~_", this_builddir, this_srcdir)
        if 'CATKIN_DPKG_BUILDPACKAGE_FLAGS' not in kwargs:
            kwargs['CATKIN_DPKG_BUILDPACKAGE_FLAGS'] = '-d;-S;-us;-uc'
        for k, v in kwargs.items():
            print("~v^v~", k, v)
            args += ["-D%s=%s" % (k, v)]
        if not 'CMAKE_INSTALL_PREFIX' in kwargs:
            if installdir is None:
                installdir = self.installdir
            args += ["-DCMAKE_INSTALL_PREFIX=%s" % (installdir)]

        if not 'CMAKE_PREFIX_PATH' in kwargs:
            if prefix_path is None:
                prefix_path = self.installdir
            args += ["-DCMAKE_PREFIX_PATH=%s" % (prefix_path)]

        if not os.path.isdir(this_builddir):
            os.makedirs(this_builddir)
        cmd = ["cmake", this_srcdir] + args
        o = expect(cmd, cwd=this_builddir)
        if (expect == succeed):
            self.assertTrue(os.path.isfile(this_builddir + "/CMakeCache.txt"))
            self.assertTrue(os.path.isfile(this_builddir + "/Makefile"))
        return o


def assert_exists(prefix, *args):
    """
    Convenience function calling exists for all files in args with
    prefix
    """
    for arg in args:
        p = os.path.join(prefix, arg)
        print("Checking for", p)
        assert os.path.exists(p), "%s doesn't exist" % p
