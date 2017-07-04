#!/usr/bin/env python

import os
import sys
import unittest
from test.utils import *
from . import network_test_utils
from network_test_utils import AbstractUnstableTest


# Tests below currently fail due to missing stack typelibxml



# @catkin_env_fixture
# def test_strangelanguage():
#     mybuild = os.path.join(TEMP_DIR, 'build_typelib')
#     if isdir(mybuild):
#         shutil.rmtree(mybuild)
#     os.makedirs(mybuild)
#     INST = os.path.join(TEMP_DIR, 'install_typelib')
#     out = cmake(CATKIN_WHITELIST_STACKS='catkin;genmsg;gentypelibxml;std_msgs',
#                 CMAKE_INSTALL_PREFIX=INST,
#                 cwd=mybuild)
#     out = succeed(["/usr/bin/make", "install"], cwd=mybuild)

#     assert_exists(INST, 'share/typelibxml/std_msgs/Float32.xml')

# @catkin_env_fixture
# def test_strangelanguage_installed():
#     mybuild = os.path.join(TEMP_DIR, 'build_typelib2')
#     if isdir(mybuild):
#         shutil.rmtree(mybuild)
#     os.makedirs(mybuild)
#     INST = os.path.join(TEMP_DIR, 'install_typelib2')
#     out = cmake(CATKIN_WHITELIST_STACKS='catkin;genmsg;gentypelibxml',
#                 CMAKE_INSTALL_PREFIX=INST,
#                 cwd=mybuild)
#     out = succeed(["/usr/bin/make", "install"], cwd=mybuild)
#     assert_exists(INST,
#                   'etc/langs/gentypelibxml',
#                   'share/gentypelibxml/cmake/gentypelibxml.cmake')

#     shutil.rmtree(mybuild)
#     os.makedirs(mybuild)

#     out = cmake(srcdir=pwd+'/src/std_msgs',
#                 cwd=mybuild,
#                 CMAKE_PREFIX_PATH=INST,
#                 CMAKE_INSTALL_PREFIX=INST)

#     out = succeed(["/usr/bin/make", "install"], cwd=mybuild)
#     assert_exists(INST,
#                   'share/typelibxml/std_msgs/Float32.xml')
