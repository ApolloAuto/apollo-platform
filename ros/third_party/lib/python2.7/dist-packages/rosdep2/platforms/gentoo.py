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
#
# Author Murph Finnicum/murph@murph.cc

### A word on atoms ###
# We'll be using 'atoms' instead of 'packages' for the majority of the gentoo installer.
# Atoms can specify a package version (either exactly, or min/max version), flags it has 
# to be built with, and even repositories it has to come from
# 
# Here are some valid atoms and their meanings:
# sed // A package named 'sed'
# sys-apps/sed // sed from the category 'sys-apps'. There can be collisions otherwise.
# sys-apps/sed::gentoo // sed from the category 'sys-apps' and the repository 'gentoo' (the default).
# >=sys-apps/sed-4 // sed of at least version 4
# sed[static,-nls] // sed built the static USE flag and withou the nls one

import os

from rospkg.os_detect import OS_GENTOO

from .source import SOURCE_INSTALLER
from ..installers import PackageManagerInstaller
from ..shell_utils import read_stdout

PORTAGE_INSTALLER = 'portage'

def register_installers(context):
    context.set_installer(PORTAGE_INSTALLER, PortageInstaller())

def register_platforms(context):
    context.add_os_installer_key(OS_GENTOO, PORTAGE_INSTALLER)
    context.add_os_installer_key(OS_GENTOO, SOURCE_INSTALLER)
    context.set_default_os_installer_key(OS_GENTOO, lambda self: PORTAGE_INSTALLER)

# Determine whether an atom is already satisfied
def portage_detect_single(atom, exec_fn = read_stdout ):
    """ 
    Check if a given atom is installed.
    
    :param exec_fn: function to execute Popen and read stdout (for testing)
    """

    std_out = exec_fn(['portageq', 'match', '/', atom])

    # TODO consdier checking the name of the package returned
    # Also, todo, figure out if just returning true if two packages are returned is cool..
    return len(std_out) >= 1

def portage_detect(atoms, exec_fn = read_stdout):
    """
    Given a list of atoms, return a list of which are already installed.

    :param exec_fn: function to execute Popen and read stdout (for testing)
    """

    # This is for testing, to make sure they're always checked in the same order
    # TODO: make testing better to not need this
    if isinstance(atoms, list):
        atoms.sort()
    
    return [a for a in atoms if portage_detect_single(a, exec_fn)]

# Check portage and needed tools for existence and compatibility
def portage_available():
    if not os.path.exists("/usr/bin/portageq"):
        return False

    if not os.path.exists("/usr/bin/emerge"):
        return False

    # We only use standard, defined portage features.
    # They work in all released versions of portage, and should work in
    # future versionf for a long time to come.
    # but .. TODO: Check versions

    return True

class PortageInstaller(PackageManagerInstaller):

    def __init__(self):
        super(PortageInstaller, self).__init__(portage_detect)
        
        
    def get_install_command(self, resolved, interactive=True, reinstall=False, quiet=False):
        atoms = self.get_packages_to_install(resolved, reinstall=reinstall)      

        cmd = self.elevate_priv(['emerge'])
        if not atoms:
            return []

        if interactive:
            cmd.append('-a')

        cmd.extend(atoms)

        return [cmd]


