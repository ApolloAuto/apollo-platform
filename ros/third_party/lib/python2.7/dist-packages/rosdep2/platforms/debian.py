#!/usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
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

# Author Tully Foote, Ken Conley

from __future__ import print_function
import sys

from rospkg.os_detect import OS_DEBIAN, OS_LINARO, OS_UBUNTU, OsDetect

from .pip import PIP_INSTALLER
from .gem import GEM_INSTALLER
from .source import SOURCE_INSTALLER
from ..installers import PackageManagerInstaller
from ..shell_utils import read_stdout

# apt package manager key
APT_INSTALLER='apt'

def register_installers(context):
    context.set_installer(APT_INSTALLER, AptInstaller())

def register_platforms(context):
    register_debian(context)
    register_linaro(context)
    register_ubuntu(context)
    
def register_debian(context):
    context.add_os_installer_key(OS_DEBIAN, APT_INSTALLER)
    context.add_os_installer_key(OS_DEBIAN, PIP_INSTALLER)
    context.add_os_installer_key(OS_DEBIAN, GEM_INSTALLER)
    context.add_os_installer_key(OS_DEBIAN, SOURCE_INSTALLER)
    context.set_default_os_installer_key(OS_DEBIAN, lambda self: APT_INSTALLER)
    context.set_os_version_type(OS_DEBIAN, OsDetect.get_codename)
    
def register_linaro(context):
    # Linaro is an alias for Ubuntu. If linaro is detected and it's not set as an override force ubuntu.
    (os_name, os_version) = context.get_os_name_and_version()
    if os_name == OS_LINARO and not context.os_override:
        print("rosdep detected OS: [%s] aliasing it to: [%s]" % (OS_LINARO, OS_UBUNTU), file=sys.stderr)
        context.set_os_override(OS_UBUNTU, context.os_detect.get_codename())

def register_ubuntu(context):
    context.add_os_installer_key(OS_UBUNTU, APT_INSTALLER)
    context.add_os_installer_key(OS_UBUNTU, PIP_INSTALLER)
    context.add_os_installer_key(OS_UBUNTU, GEM_INSTALLER)
    context.add_os_installer_key(OS_UBUNTU, SOURCE_INSTALLER)
    context.set_default_os_installer_key(OS_UBUNTU, lambda self: APT_INSTALLER)
    context.set_os_version_type(OS_UBUNTU, OsDetect.get_codename)

def dpkg_detect(pkgs, exec_fn=None):
    """ 
    Given a list of package, return the list of installed packages.

    :param exec_fn: function to execute Popen and read stdout (for testing)
    """
    ret_list = []
    # this is mainly a hack to support version locking for eigen.
    # we strip version-locking syntax, e.g. libeigen3-dev=3.0.1-*.
    # our query does not do the validation on the version itself.
    version_lock_map = {}
    for p in pkgs:
        if '=' in p:
            version_lock_map[p.split('=')[0]] = p
        else:
            version_lock_map[p] = p
    cmd = ['dpkg-query', '-W', '-f=\'${Package} ${Status}\n\'']
    cmd.extend(version_lock_map.keys())

    if exec_fn is None:
        exec_fn = read_stdout
    std_out = exec_fn(cmd)
    std_out = std_out.replace('\'','')
    pkg_list = std_out.split('\n')
    for pkg in pkg_list:
        pkg_row = pkg.split()
        if len(pkg_row) == 4 and (pkg_row[3] =='installed'):
            ret_list.append( pkg_row[0])
    return [version_lock_map[r] for r in ret_list]


class AptInstaller(PackageManagerInstaller):
    """
    An implementation of the Installer for use on debian style
    systems.
    """
    def __init__(self):
        super(AptInstaller, self).__init__(dpkg_detect)

    def get_install_command(self, resolved, interactive=True, reinstall=False, quiet=False):
        packages = self.get_packages_to_install(resolved, reinstall=reinstall)
        if not packages:
            return []
        if not interactive and quiet:
            return [self.elevate_priv(['apt-get', 'install', '-y', '-qq', p]) for p in packages]
        elif quiet:
            return [self.elevate_priv(['apt-get', 'install', '-qq', p]) for p in packages]
        if not interactive:
            return [self.elevate_priv(['apt-get', 'install', '-y', p]) for p in packages]
        else:
            return [self.elevate_priv(['apt-get', 'install', p]) for p in packages]
