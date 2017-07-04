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

from rospkg.os_detect import OS_OPENSUSE

from .source import SOURCE_INSTALLER
from ..installers import PackageManagerInstaller

# zypper package manager key
ZYPPER_INSTALLER='zypper'

def register_installers(context):
    context.set_installer(ZYPPER_INSTALLER, ZypperInstaller())

def register_platforms(context):
    context.add_os_installer_key(OS_OPENSUSE, SOURCE_INSTALLER)
    context.add_os_installer_key(OS_OPENSUSE, ZYPPER_INSTALLER)
    context.set_default_os_installer_key(OS_OPENSUSE, lambda self: ZYPPER_INSTALLER)
    
def rpm_detect(packages):
    installed = []
    for p in packages:
        if not subprocess.call(['rpm', '-q', p]):
            installed.append(p)
    return installed

class ZypperInstaller(PackageManagerInstaller):
    """
    This class provides the functions for installing using zypper.
    """

    def __init__(self):
        super(ZypperInstaller, self).__init__(rpm_detect)

    def get_install_command(self, resolved, interactive=True, reinstall=False, quiet=False):
        packages = self.get_packages_to_install(resolved, reinstall=reinstall)
        if not packages:
            return []
        if not interactive:
            return [self.elevate_priv(['zypper', 'install', '-yl'])+packages]
        else:
            return [self.elevate_priv(['zypper', 'install'])+packages]
