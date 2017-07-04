#!/usr/bin/env python
# Copyright (c) 2010, Willow Garage, Inc.
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

# Original from cygwin.py by Tingfan Wu tingfan@gmail.com
# Modified for FreeBSD by Rene Ladan rene@freebsd.org

import os
import subprocess

from rospkg.os_detect import OS_FREEBSD

from .source import SOURCE_INSTALLER
from ..installers import Installer

PKG_ADD_INSTALLER = 'pkg_add'

def register_installers(context):
    context.set_installer(PKG_ADD_INSTALLER, PkgAddInstaller())
    
def register_platforms(context):
    context.add_os_installer_key(OS_FREEBSD, SOURCE_INSTALLER)
    context.add_os_installer_key(OS_FREEBSD, PKG_ADD_INSTALLER)
    context.set_default_os_installer_key(OS_FREEBSD, lambda self: PKG_ADD_INSTALLER)

def pkg_info_detect_single(p):
    if p == "builtin":
        return True
    # The next code is a lot of hassle, but there is no
    # better way in FreeBSD using just the base tools
    portname = p
    if p == "gtk20":
        portname = "gtk-2.\*"
    elif p == "py-gtk2":
        portname = "py27-gtk-2.\*"
    elif p[:9] in ["autoconf2", "automake1"]:
        portname = p[:8] + "-" + p[8] + "." + p[9:] + "\*"
    elif p[:3] == "py-":
        portname = "py27-" + p[3:] + "\*"
    else:
        portname = p + "-\*"
    pop = subprocess.Popen("/usr/sbin/pkg_info -qE " + portname, shell=True)
    return os.waitpid(pop.pid, 0)[1] == 0 # pkg_info -E returns 0 if pkg installed, 1 if not

def pkg_info_detect(packages):
    return [p for p in packages if pkg_info_detect_single(p)]

class PkgAddInstaller(Installer):
    """
    An implementation of the Installer for use on FreeBSD-style
    systems.
    """

    def __init__(self):
        super(PkgAddInstaller, self).__init__(pkg_info_detect)

    def get_install_command(self, resolved, interactive=True, reinstall=False, quiet=False):
        packages = self.get_packages_to_install(resolved, reinstall=reinstall)        
        if not packages:
            return []
        else:
            #pkg_add does not have a non-interactive command
            return [self.elevate_priv(['/usr/sbin/pkg_add', '-r'])+packages]
