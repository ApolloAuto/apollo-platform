# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author Dirk Thomas/dthomas@willowgarage.com

"""
API provided for rospack to determine if a dependency
is a ROSpackage or a system dependency
"""

from __future__ import print_function

import subprocess

from .main import _get_default_RosdepLookup
from .rospkg_loader import DEFAULT_VIEW_KEY
from .sources_list import get_sources_cache_dir


def call_pkg_config(option, pkg_name):
    try:
        value = subprocess.check_output(['pkg-config', option, pkg_name])
        return value.strip()
    except subprocess.CalledProcessError:
        return None


def init_rospack_interface():
    class Options(object):
        def __init__(self):
            self.os_override = None
            self.sources_cache_dir = get_sources_cache_dir()
            self.verbose = False
    lookup = _get_default_RosdepLookup(Options())
    return lookup.get_rosdep_view(DEFAULT_VIEW_KEY)


def is_view_empty(view):
    return len(view.rosdep_defs) == 0


def is_ros_package(view, rosdep_name):
    return _ros_flag(view, rosdep_name, True)


def is_system_dependency(view, rosdep_name):
    return _ros_flag(view, rosdep_name, False)


def _ros_flag(view, rosdep_name, value):
    try:
        d = view.lookup(rosdep_name)
    except KeyError:
        return False
    ros_flag = '_is_ros' in d.data.keys()
    return ros_flag == value
