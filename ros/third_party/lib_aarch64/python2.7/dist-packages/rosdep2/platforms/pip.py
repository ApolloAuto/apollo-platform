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

from __future__ import print_function

import subprocess

from ..core import InstallFailed
from ..installers import PackageManagerInstaller
from ..shell_utils import read_stdout

# pip package manager key
PIP_INSTALLER = 'pip'


def register_installers(context):
    context.set_installer(PIP_INSTALLER, PipInstaller())


def is_pip_installed():
    try:
        subprocess.Popen(['pip'], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
        return True
    except OSError:
        return False


def pip_detect(pkgs, exec_fn=None):
    """
    Given a list of package, return the list of installed packages.

    :param exec_fn: function to execute Popen and read stdout (for testing)
    """
    fallback_to_pip_show = False
    if exec_fn is None:
        exec_fn = read_stdout
        fallback_to_pip_show = True
    pkg_list = exec_fn(['pip', 'freeze']).split('\n')

    ret_list = []
    for pkg in pkg_list:
        pkg_row = pkg.split("==")
        if pkg_row[0] in pkgs:
            ret_list.append(pkg_row[0])

    # Try to detect with the return code of `pip show`.
    # This can show the existance of things like `argparse` which
    # otherwise do not show up.
    # See:
    #   https://github.com/pypa/pip/issues/1570#issuecomment-71111030
    if fallback_to_pip_show:
        for pkg in [p for p in pkgs if p not in ret_list]:
            # does not see retcode but stdout for old pip to check if installed
            proc = subprocess.Popen(
                ['pip', 'show', pkg],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT
            )
            output, _ = proc.communicate()
            output = output.strip()
            if proc.returncode == 0 and output:
                # `pip show` detected it, add it to the list.
                ret_list.append(pkg)

    return ret_list


class PipInstaller(PackageManagerInstaller):
    """
    :class:`Installer` support for pip.
    """

    def __init__(self):
        super(PipInstaller, self).__init__(pip_detect, supports_depends=True)

    def get_install_command(self, resolved, interactive=True, reinstall=False, quiet=False):
        if not is_pip_installed():
            raise InstallFailed((PIP_INSTALLER, "pip is not installed"))
        packages = self.get_packages_to_install(resolved, reinstall=reinstall)
        if not packages:
            return []
        else:
            return [self.elevate_priv(['pip', 'install', '-U', p]) for p in packages]
