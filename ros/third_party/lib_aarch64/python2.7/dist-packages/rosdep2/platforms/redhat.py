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

# Author Tully Foote/tfoote@willowgarage.com

import subprocess

from rospkg.os_detect import OS_RHEL, OS_FEDORA

from .pip import PIP_INSTALLER
from .source import SOURCE_INSTALLER
from ..core import rd_debug
from ..installers import PackageManagerInstaller
from ..shell_utils import read_stdout

# dnf package manager key
DNF_INSTALLER='dnf'

# yum package manager key
YUM_INSTALLER='yum'

def register_installers(context):
    context.set_installer(DNF_INSTALLER, DnfInstaller())
    context.set_installer(YUM_INSTALLER, YumInstaller())

def register_platforms(context):
    register_fedora(context)
    register_rhel(context)
    
def register_fedora(context):
    context.add_os_installer_key(OS_FEDORA, PIP_INSTALLER)
    context.add_os_installer_key(OS_FEDORA, DNF_INSTALLER)
    context.add_os_installer_key(OS_FEDORA, YUM_INSTALLER)
    context.add_os_installer_key(OS_FEDORA, SOURCE_INSTALLER)
    context.set_default_os_installer_key(OS_FEDORA, lambda self: DNF_INSTALLER if self.get_version().isdigit() and int(self.get_version()) > 21 else YUM_INSTALLER)
    context.set_os_version_type(OS_FEDORA, lambda self: self.get_version() if self.get_version().isdigit() and int(self.get_version()) > 20 else self.get_codename())

def register_rhel(context):
    context.add_os_installer_key(OS_RHEL, PIP_INSTALLER)
    context.add_os_installer_key(OS_RHEL, YUM_INSTALLER)
    context.add_os_installer_key(OS_RHEL, SOURCE_INSTALLER)
    context.set_default_os_installer_key(OS_RHEL, lambda self: YUM_INSTALLER)

def rpm_detect_py(packages):
    ret_list = []
    import rpm
    ts = rpm.TransactionSet()
    for raw_req in packages:
        req = rpm_expand_py(raw_req)
        rpms = ts.dbMatch(rpm.RPMTAG_PROVIDES, req)
        if len(rpms) > 0:
            ret_list += [raw_req]
    return ret_list

def rpm_detect_cmd(raw_packages, exec_fn=None):
    ret_list = []

    if exec_fn is None:
        exec_fn = read_stdout

    packages = [rpm_expand_cmd(package, exec_fn) for package in raw_packages]

    cmd = ['rpm', '-q', '--whatprovides', '--qf', '[%{PROVIDES}\n]']
    cmd.extend(packages)

    std_out = exec_fn(cmd)
    out_lines = std_out.split()
    for index, package in enumerate(packages):
        if package in out_lines:
            ret_list.append(raw_packages[index])
    return ret_list

def rpm_detect(packages, exec_fn=None):
    try:
        return rpm_detect_py(packages)
    except ImportError:
        return rpm_detect_cmd(packages, exec_fn)

def rpm_expand_py(macro):
    import rpm
    expanded = rpm.expandMacro(macro)
    rd_debug('Expanded rpm macro in \'%s\' to \'%s\'' % (macro, expanded))
    return expanded

def rpm_expand_cmd(macro, exec_fn=None):
    cmd = ['rpm', '-E', macro]

    if exec_fn is None:
        exec_fn = read_stdout

    expanded = exec_fn(cmd).strip()
    rd_debug('Expanded rpm macro in \'%s\' to \'%s\'' % (macro, expanded))
    return expanded

def rpm_expand(package, exec_fn=None):
    if not '%' in package:
        return package
    try:
        return rpm_expand_py(package)
    except ImportError:
        return rpm_expand_cmd(package, exec_fn)

class DnfInstaller(PackageManagerInstaller):
    """
    This class provides the functions for installing using dnf
    it's methods partially implement the Rosdep OS api to complement
    the roslib.OSDetect API.
    """

    def __init__(self):
        super(DnfInstaller, self).__init__(rpm_detect)

    def get_install_command(self, resolved, interactive=True, reinstall=False, quiet=False):
        raw_packages = self.get_packages_to_install(resolved, reinstall=reinstall)
        packages = [rpm_expand(package) for package in raw_packages]

        if not packages:
            return []
        elif not interactive and quiet:
            return [self.elevate_priv(['dnf', '--assumeyes', '--quiet', 'install']) + packages]
        elif quiet:
            return [self.elevate_priv(['dnf', '--quiet', 'install']) + packages]
        elif not interactive:
            return [self.elevate_priv(['dnf', '--assumeyes', 'install']) + packages]
        else:
            return [self.elevate_priv(['dnf', 'install']) + packages]


class YumInstaller(PackageManagerInstaller):
    """
    This class provides the functions for installing using yum
    it's methods partially implement the Rosdep OS api to complement
    the roslib.OSDetect API.
    """

    def __init__(self):
        super(YumInstaller, self).__init__(rpm_detect)

    def get_install_command(self, resolved, interactive=True, reinstall=False, quiet=False):
        raw_packages = self.get_packages_to_install(resolved, reinstall=reinstall)
        packages = [rpm_expand(package) for package in raw_packages]

        if not packages:
            return []
        elif not interactive and quiet:
            return [self.elevate_priv(['yum', '--assumeyes', '--quiet', '--skip-broken', 'install']) + packages]
        elif quiet:
            return [self.elevate_priv(['yum', '--quiet', '--skip-broken', 'install']) + packages]
        elif not interactive:
            return [self.elevate_priv(['yum', '--assumeyes', '--skip-broken', 'install']) + packages]
        else:
            return [self.elevate_priv(['yum', '--skip-broken', 'install']) + packages]
