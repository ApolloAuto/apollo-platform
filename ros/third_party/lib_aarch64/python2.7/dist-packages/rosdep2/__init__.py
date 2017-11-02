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

# Author Tully Foote/tfoote@willowgarage.com, Ken Conley/kwc@willowgarage.com

"""
rosdep library and command-line tool
"""

from __future__ import print_function

from ._version import __version__

import sys

from .installers import InstallerContext, Installer, \
        PackageManagerInstaller
from .core import RosdepInternalError, InstallFailed, UnsupportedOs, \
        InvalidData, DownloadFailure
from .model import RosdepDatabase, RosdepDatabaseEntry
from .lookup import RosdepDefinition, RosdepView, RosdepLookup, \
        ResolutionError
from .loader import RosdepLoader

# don't let import error take down code as when attempting to compute version number
try:
    from .rospkg_loader import RosPkgLoader
except ImportError:
    print("Cannot import rospkg, rosdep will not function properly", 
            file=sys.stderr)


def create_default_installer_context(verbose=False):
    from .platforms import arch
    from .platforms import cygwin
    from .platforms import debian
    from .platforms import gentoo
    from .platforms import opensuse
    from .platforms import osx
    from .platforms import pip
    from .platforms import gem
    from .platforms import redhat
    from .platforms import source

    platform_mods = [arch, cygwin, debian, gentoo, opensuse, osx, redhat]
    installer_mods = [source, pip, gem] + platform_mods

    context = InstallerContext()
    context.set_verbose(verbose)

    # setup installers
    for m in installer_mods:
        if verbose:
            print("registering installers for %s"%(m.__name__))
        m.register_installers(context)

    # setup platforms
    for m in platform_mods:
        if verbose:
            print("registering platforms for %s"%(m.__name__))
        m.register_platforms(context)

    return context

from . import gbpdistro_support
gbpdistro_support.create_default_installer_context = create_default_installer_context


#TODO: this was partially abstracted from main() for another library,
# but it turned out to be unnecessary. Not sure it's worth maintaining
# separately, especially in the top-level module.
def get_default_installer(installer_context=None, verbose=False):
    """
    Based on the active OS and installer context configuration, get
    the installer to use and the necessary configuration state
    (installer keys, OS name/version).
    
    :returns: installer, installer_keys, default_key, os_name, os_version. 
    """
    if installer_context is None:
        installer_context = create_default_installer_context(verbose=verbose)

    os_name, os_version = installer_context.get_os_name_and_version()
    try:
        installer_keys = installer_context.get_os_installer_keys(os_name)
        default_key = installer_context.get_default_os_installer_key(os_name)
    except KeyError:
        raise UnsupportedOs(os_name, installer_context.get_os_keys())
    installer = installer_context.get_installer(default_key)
    return installer, installer_keys, default_key, os_name, os_version

__all__ = ['InstallerContext', 'Installer', 'PackageManagerInstaller',
        'RosdepInternalError', 'InstallFailed', 'UnsupportedOs', 'InvalidData',
        'DownloadFailure',
        'RosdepDatabase', 'RosdepDatabaseEntry',
        'RosdepDefinition', 'RosdepView', 'RosdepLookup', 'ResolutionError',
        'RosdepLoader', 'RosPkgLoader',
        'get_default_installer', 
        'create_default_installer_context',
        ]
