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

# Author Tully Foote/tfoote@willowgarage.com, Ken Conley

import subprocess
import json
import sys
import traceback

from rospkg.os_detect import OS_OSX, OsDetect

from ..core import InstallFailed, RosdepInternalError, InvalidData
from .pip import PIP_INSTALLER
from .source import SOURCE_INSTALLER
from ..installers import PackageManagerInstaller
from ..shell_utils import read_stdout

# add additional os names for brew, macports (TODO)
OSXBREW_OS_NAME = 'osxbrew'

BREW_INSTALLER = 'homebrew'
MACPORTS_INSTALLER = 'macports'

#py3k
try:
    _basestring = basestring
except NameError:
    _basestring = str

def register_installers(context):
    context.set_installer(MACPORTS_INSTALLER, MacportsInstaller())
    context.set_installer(BREW_INSTALLER, HomebrewInstaller())
    
def register_platforms(context):
    context.add_os_installer_key(OS_OSX, BREW_INSTALLER)
    context.add_os_installer_key(OS_OSX, MACPORTS_INSTALLER)
    context.add_os_installer_key(OS_OSX, PIP_INSTALLER)
    context.add_os_installer_key(OS_OSX, SOURCE_INSTALLER)
    context.set_default_os_installer_key(OS_OSX, lambda self: BREW_INSTALLER)
    context.set_os_version_type(OS_OSX, OsDetect.get_codename)

def is_port_installed():
    try:
        subprocess.Popen(['port'], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
        return True
    except OSError:
        return False
    
def port_detect(pkgs, exec_fn=None):
    ret_list = []
    if not is_port_installed():
        return ret_list
    if exec_fn is None:
        exec_fn = read_stdout
    std_out = exec_fn(['port', 'installed']+pkgs)
    for pkg in std_out.split('\n'):
        pkg_row = pkg.split()
        if len(pkg_row) == 3 and pkg_row[0] in pkgs and pkg_row[2] =='(active)':
            ret_list.append(pkg_row[0])
    return ret_list

class MacportsInstaller(PackageManagerInstaller):
    """ 
    An implementation of the :class:`Installer` API for use on
    macports systems.
    """
    def __init__(self):
        super(MacportsInstaller, self).__init__(port_detect)

    def get_install_command(self, resolved, interactive=True, reinstall=False):
        if not is_port_installed():
            raise InstallFailed((MACPORTS_INSTALLER, 'MacPorts is not installed'))
        packages = self.get_packages_to_install(resolved)
        if not packages:
            return []
        else:
            #TODO: interactive
            return [self.elevate_priv(['port', 'install', p]) for p in packages]

def is_brew_installed():
    try:
        subprocess.Popen(['brew'], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
        return True
    except OSError:
        return False


class HomebrewResolution(object):

    """Resolution information for a single package of a Homebrew rosdep."""

    def __init__(self, package, install_flags, options):
        """
        :param package: Homebrew package name, possibly fully qualified
            with tap.
        :param install_flags: List of strings of additional flags for
            ``brew install`` and ``brew deps`` command which are not
            options (e.g. ``--HEAD``)
        :param options: List of strings of options for the homebrew
            package.
        """
        self.package = package
        self.install_flags = install_flags
        self.options = options

    def __eq__(self, other):
        return other.package == self.package and \
               other.install_flags == self.install_flags and \
               other.options == self.options

    def __hash__(self):
        return hash((
            type(self),
            self.package,
            tuple(self.install_flags),
            tuple(self.options)))

    def __str__(self):
        return ' '.join(self.to_list())

    def to_list(self):
        return [self.package] + self.install_flags + self.options


def brew_strip_pkg_name(package):
    """Strip the tap information of a fully qualified package name.

    :returns: Unqualified package name. E.g. 'foo-pkg' for input
        'ros/hydro/foo-pkg'
    """
    return package.split('/')[-1]


def brew_detect(resolved, exec_fn=None):
    """Given a list of resolutions, return the list of installed resolutions.

    :param resolved: List of HomebrewResolution objects
    :returns: Filtered list of HomebrewResolution objects
    """
    if exec_fn is None:
        exec_fn = read_stdout
    std_out = exec_fn(['brew', 'list'])
    installed_formulae = std_out.split()

    def is_installed(r):
        # TODO: Does not check installed version (stable, devel, HEAD)
        # TODO: Does not check origin (Tap) of formula
        # TODO: Does not handle excluding options (e.g. specifying
        #       --without-foo for --with-foo option)

        # fast fail with a quick check first, then slower check if
        # really linked and for options
        if not brew_strip_pkg_name(r.package) in installed_formulae:
            return False

        std_out = exec_fn(['brew', 'info', r.package, '--json=v1'])
        try:
            pkg_info = json.loads(std_out)
            pkg_info = pkg_info[0]
            linked_version = pkg_info['linked_keg']
            if not linked_version:
                return False
            for spec in pkg_info['installed']:
                if spec['version'] == linked_version:
                    installed_options = spec['used_options']
                    break
        except (ValueError, TypeError):
            e_type, e, tb = sys.exc_info()
            raise RosdepInternalError(
                e, """Error while parsing brew info for '{0}'
 * Output of `brew info {0} --json=v1`:
 {1}
 * Error while parsing:
 {2}""".format(r.package, std_out, "".join(traceback.format_exception(e_type, e, tb))))

        if set(r.options) <= set(installed_options):
            return True
        else:
            return False

    # preserve order
    return list(filter(is_installed, resolved))


class HomebrewInstaller(PackageManagerInstaller):

    """
    An implementation of Installer for use on homebrew systems.

    Some examples for supported rosdep specifications:

    # Example 1: flat list of options if only one package defined.
    foo:
        osx:
            homebrew:
                depends: [bar]
                options: [--with-quux, --with-quax]
                packages: [foo-pkg]

    # Example 2: list of list of options for multiple packages
    bar:
        osx:
            homebrew:
                options: [[], [--with-quux]]
                packages: [bar-pkg, bar-pkg-dev]

    # Example 3: list of options can be shorter than list of packages (filling
    # up with empty options)
    baz:
        osx:
            homebrew:
                options: [[--with-quax]]
                packages: [baz-pkg, baz-pkg-dev]

    # Example 4: No options is fine.
    buz:
        osx:
            homebrew:
                packages: [buz-pkg]

    ``install_flags`` are handled analogously to ``options``.
    """

    def __init__(self):
        super(HomebrewInstaller, self).__init__(brew_detect, supports_depends=True)
        self.as_root = False

    def resolve(self, rosdep_args):
        """
        See :meth:`Installer.resolve()`
        """

        def coerce_to_list(options):
            if isinstance(options, list):
                return options
            elif isinstance(options, _basestring):
                return options.split()
            else:
                raise InvalidData("Expected list or string for options '%s'" % options)

        def handle_options(options):
            # if only one package is specified we allow a flat list of options
            if len(packages) == 1 and options and not isinstance(options[0],list):
                options = [options]
            else:
                options = list(map(coerce_to_list, options))

            # make sure options is a list of list of strings
            try:
                valid = all([isinstance(x, _basestring) for l in options for x in l])
            except Exception as e:
                raise InvalidData("Invalid list of options '%s', error: %s" % (options, e))
            else:
                if not valid:
                    raise InvalidData("Invalid list of options '%s'" % options)

            # allow only fewer or equal number of option lists
            if len(options) > len(packages):
                raise InvalidData("More options '%s' than packages '%s'" % (options, packages))
            else:
                options.extend([[]] * (len(packages) - len(options)))

            return options

        packages = super(HomebrewInstaller, self).resolve(rosdep_args)
        resolution = []
        if packages:
            options = []
            install_flags = []
            if type(rosdep_args) == dict:
                options = coerce_to_list(rosdep_args.get("options", []))
                install_flags = coerce_to_list(rosdep_args.get("install_flags", []))

            options = handle_options(options)
            install_flags = handle_options(install_flags)

            # packages, options and install_flags now have the same length
            resolution = map(HomebrewResolution, packages, install_flags, options)
        return resolution

    def get_install_command(self, resolved, interactive=True, reinstall=False, quiet=False):
        # TODO: We should somehow inform the user that we uninstall all versions
        #       of packages and do not keep track of which options have been
        #       activated. Then again, maybe not this would be the
        #       responsibility of the user to before or not use --reinstall.

        if not is_brew_installed():
            raise InstallFailed((BREW_INSTALLER, 'Homebrew is not installed'))
        resolved = self.get_packages_to_install(resolved, reinstall=reinstall)
        resolved = self.remove_duplicate_dependencies(resolved)
        # interactive switch doesn't matter
        if reinstall:
            commands = []
            for r in resolved:
                # --force uninstalls all versions of that package
                commands.append(self.elevate_priv(['brew', 'uninstall', '--force', r.package]))
                commands.append(self.elevate_priv(['brew', 'install'] + r.to_list()))
            return commands
        else:
            return [self.elevate_priv(['brew', 'install'] + r.to_list()) for r in resolved]

    def remove_duplicate_dependencies(self, resolved):
        # TODO: we do not look at options here, however the install check later
        #       will inform use if installed options are not appropriate
        # TODO: we comapre unqualified package names, ignoring the specifed tap

        if not is_brew_installed():
            raise InstallFailed((BREW_INSTALLER, 'Homebrew is not installed'))

        # we'll remove dependencies from this copy and return it
        resolved_copy = list(resolved)

        # find all dependencies for each package
        for r in resolved:
            sub_command = ['brew', 'deps'] + r.to_list()
            output = subprocess.Popen(sub_command, stdout=subprocess.PIPE).communicate()[0]
            deps = output.split()
            for d in deps:
                # remove duplicate dependency from package list
                for other in resolved_copy:
                    if brew_strip_pkg_name(other.package) == brew_strip_pkg_name(d):
                        resolved_copy.remove(other)
        return resolved_copy
