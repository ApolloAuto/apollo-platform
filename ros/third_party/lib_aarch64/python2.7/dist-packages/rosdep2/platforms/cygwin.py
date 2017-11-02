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

# Tingfan Wu tingfan@gmail.com

from __future__ import print_function

from rospkg.os_detect import OS_CYGWIN

from .source import SOURCE_INSTALLER

from ..installers import PackageManagerInstaller
from ..shell_utils import read_stdout

APT_CYG_INSTALLER = 'apt-cyg'

def register_installers(context):
    context.set_installer(APT_CYG_INSTALLER, AptCygInstaller())
    
def register_platforms(context):
    context.add_os_installer_key(OS_CYGWIN, SOURCE_INSTALLER)
    context.add_os_installer_key(OS_CYGWIN, APT_CYG_INSTALLER)
    context.set_default_os_installer_key(OS_CYGWIN, lambda self: APT_CYG_INSTALLER)

def cygcheck_detect_single(p):
    std_out = read_stdout(['cygcheck', '-c', p])
    return std_out.count("OK") > 0

def cygcheck_detect(packages):
    return [p for p in packages if cygcheck_detect_single(p)]

class AptCygInstaller(PackageManagerInstaller):
    """
    An implementation of the :class:`Installer` for use on
    cygwin-style systems.
    """

    def __init__(self):
        super(AptCygInstaller, self).__init__(cygcheck_detect)
        self.as_root = False
        self.sudo_command = 'cygstart --action=runas'

    def get_install_command(self, resolved, interactive=True, reinstall=False, quiet=False):
        packages = self.get_packages_to_install(resolved, reinstall=reinstall)        
        #TODO: interactive
        if not packages:
            return []
        else:
            return [self.elevate_priv(['apt-cyg', '-m', 'ftp://sourceware.org/pub/cygwinports', 'install'])+packages]

if __name__ == '__main__':
    print("test cygcheck_detect(true)", cygcheck_detect('cygwin'))
