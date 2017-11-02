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

from __future__ import print_function

import subprocess
import traceback

from rospkg.os_detect import OsDetect

from .core import rd_debug, RosdepInternalError, InstallFailed, print_bold, InvalidData

# kwc: InstallerContext is basically just a bunch of dictionaries with
# defined lookup methods.  It really encompasses two facets of a
# rosdep configuration: the pluggable nature of installers and
# platforms, as well as the resolution of the operating system for a
# specific machine.  It is possible to decouple those two notions,
# though there are some touch points over how this interfaces with the
# rospkg.os_detect library, i.e. how platforms can tweak these
# detectors and how the higher-level APIs can override them.
class InstallerContext(object):
    """
    :class:`InstallerContext` manages the context of execution for rosdep as it
    relates to the installers, OS detectors, and other extensible
    APIs.
    """
    
    def __init__(self, os_detect=None):
        """
        :param os_detect: (optional)
        :class:`rospkg.os_detect.OsDetect` instance to use for
          detecting platforms.  If `None`, default instance will be
          used.
        """
        # platform configuration
        self.installers = {}
        self.os_installers = {}
        self.default_os_installer = {}

        # stores configuration of which value to use for the OS version key (version number or codename)
        self.os_version_type = {}

        # OS detection and override
        if os_detect is None:
            os_detect = OsDetect()
        self.os_detect = os_detect
        self.os_override = None

        self.verbose = False
        
    def set_verbose(self, verbose):
        self.verbose = verbose
        
    def set_os_override(self, os_name, os_version):
        """
        Override the OS detector with *os_name* and *os_version*.  See
        :meth:`InstallerContext.detect_os`.

        :param os_name: OS name value to use, ``str``
        :param os_version: OS version value to use, ``str``
        """
        if self.verbose:
            print("overriding OS to [%s:%s]"%(os_name, os_version))
        self.os_override = os_name, os_version

    def get_os_version_type(self, os_name):
        return self.os_version_type.get(os_name, OsDetect.get_version)

    def set_os_version_type(self, os_name, version_type):
        if not hasattr(version_type, '__call__'):
            raise ValueError("version type should be a method")
        self.os_version_type[os_name] = version_type
        
    def get_os_name_and_version(self):
        """
        Get the OS name and version key to use for resolution and
        installation.  This will be the detected OS name/version
        unless :meth:`InstallerContext.set_os_override()` has been
        called.

        :returns: (os_name, os_version), ``(str, str)``
        """
        if self.os_override:
            return self.os_override
        else:
            os_name = self.os_detect.get_name()
            os_key = self.get_os_version_type(os_name)
            os_version = os_key(self.os_detect)
            return os_name, os_version
        
    def get_os_detect(self):
        """
        :returns os_detect: :class:`OsDetect` instance used for
          detecting platforms.
        """
        return self.os_detect

    def set_installer(self, installer_key, installer):
        """
        Set the installer to use for *installer_key*.  This will
        replace any existing installer associated with the key.
        *installer_key* should be the same key used for the
        ``rosdep.yaml`` package manager key.  If *installer* is
        ``None``, this will delete any existing associated installer
        from this context.

        :param installer_key: key/name to associate with installer, ``str``
        :param installer: :class:`Installer` implementation, ``class``.
        :raises: :exc:`TypeError` if *installer* is not a subclass of
          :class:`Installer`
        """
        if installer is None:
            del self.installers[installer_key]
            return
        if not isinstance(installer, Installer):
            raise TypeError("installer must be a instance of Installer")
        if self.verbose:
            print("registering installer [%s]"%(installer_key))
        self.installers[installer_key] = installer
        
    def get_installer(self, installer_key):
        """
        :returns: :class:`Installer` class associated with *installer_key*.
        :raises: :exc:`KeyError` If not associated installer
        :raises: :exc:`InstallFailed` If installer cannot produce an install command (e.g. if installer is not installed)
        """
        return self.installers[installer_key]

    def get_installer_keys(self):
        """
        :returns: list of registered installer keys
        """
        return self.installers.keys()

    def get_os_keys(self):
        """
        :returns: list of OS keys that have registered with this context, ``[str]``
        """
        return self.os_installers.keys()
    
    def add_os_installer_key(self, os_key, installer_key):
        """
        Register an installer for the specified OS.  This will fail
        with a :exc:`KeyError` if no :class:`Installer` can be found
        with the associated *installer_key*.
        
        :param os_key: Key for OS
        :param installer_key: Key for installer to add to OS
        :raises: :exc:`KeyError`: if installer for *installer_key*
          is not set.
        """
        # validate, will throw KeyError
        self.get_installer(installer_key)
        if self.verbose:
            print("add installer [%s] to OS [%s]"%(installer_key, os_key))
        if os_key in self.os_installers:
            self.os_installers[os_key].append(installer_key)
        else:
            self.os_installers[os_key] = [installer_key]

    def get_os_installer_keys(self, os_key):
        """
        Get list of installer keys registered for the specified OS.
        These keys can be resolved by calling
        :meth:`InstallerContext.get_installer`.
        
        :param os_key: Key for OS
        :raises: :exc:`KeyError`: if no information for OS *os_key* is registered.
        """
        if os_key in self.os_installers:
            return self.os_installers[os_key][:]
        else:
            raise KeyError(os_key)

    def set_default_os_installer_key(self, os_key, installer_key):
        """
        Set the default OS installer to use for OS.
        :meth:`InstallerContext.add_os_installer` must have previously
        been called with the same arguments.

        :param os_key: Key for OS
        :param installer_key: Key for installer to add to OS
        :raises: :exc:`KeyError`: if installer for *installer_key*
          is not set or if OS for *os_key* has no associated installers.
        """
        if not os_key in self.os_installers:
            raise KeyError("unknown OS: %s"%(os_key))
        if not hasattr(installer_key, '__call__'):
            raise ValueError("version type should be a method")
        if not installer_key(self.os_detect) in self.os_installers[os_key]:
            raise KeyError("installer [%s] is not associated with OS [%s]. call add_os_installer_key() first"%(installer_key(self.os_detect), os_key))
        if self.verbose:
            print("set default installer for OS [%s]"%(os_key,))
        self.default_os_installer[os_key] = installer_key

    def get_default_os_installer_key(self, os_key):
        """
        Get the default OS installer key to use for OS, or ``None`` if
        there is no default.

        :param os_key: Key for OS
        :returns: :class:`Installer`
        :raises: :exc:`KeyError`: if no information for OS *os_key* is registered.
        """
        if not os_key in self.os_installers:
            raise KeyError("unknown OS: %s"%(os_key))
        try:
            installer_key = self.default_os_installer[os_key](self.os_detect)
            if not installer_key in self.os_installers[os_key]:
                raise KeyError("installer [%s] is not associated with OS [%s]. call add_os_installer_key() first"%(installer_key, os_key))
            # validate, will throw KeyError
            self.get_installer(installer_key)
            return installer_key
        except KeyError:
            return None

class Installer(object):
    """
    The :class:`Installer` API is designed around opaque *resolved*
    parameters. These parameters can be any type of sequence object,
    but they must obey set arithmetic.  They should also implement
    ``__str__()`` methods so they can be pretty printed.
    """

    def is_installed(self, resolved_item):
        """
        :param resolved: resolved installation item. NOTE: this is a single item,
          not a list of items like the other APIs, ``opaque``.
        :returns: ``True`` if all of the *resolved* items are installed on
          the local system
        """
        raise NotImplementedError("is_installed", resolved_item) 
        
    def get_install_command(self, resolved, interactive=True, reinstall=False, quiet=False):
        """
        :param resolved: list of resolved installation items, ``[opaque]``
        :param interactive: If `False`, disable interactive prompts,
          e.g. Pass through ``-y`` or equivalant to package manager.
        :param reinstall: If `True`, install everything even if already installed
        """
        raise NotImplementedError("get_package_install_command", resolved, interactive, reinstall, quiet)

    def get_depends(self, rosdep_args): 
        """ 
        :returns: list of dependencies on other rosdep keys.  Only
          necessary if the package manager doesn't handle
          dependencies.
        """
        return [] # Default return empty list

    def resolve(self, rosdep_args_dict):
        """
        :param rosdep_args_dict: argument dictionary to the rosdep rule for this package manager
        :returns: [resolutions].  resolved objects should be printable to a user, but are otherwise opaque.
        """
        raise NotImplementedError("Base class resolve", rosdep_args_dict)

    def unique(self, *resolved_rules):
        """
        Combine the resolved rules into a unique list.  This
        is meant to combine the results of multiple calls to
        :meth:`PackageManagerInstaller.resolve`.

        Example::

            resolved1 = installer.resolve(args1)
            resolved2 = installer.resolve(args2)
            resolved = installer.unique(resolved1, resolved2)

        :param *resolved_rules: resolved arguments.  Resolved
          arguments must all be from this :class:`Installer` instance.
        """
        raise NotImplementedError("Base class unique", resolved_rules)
    
class PackageManagerInstaller(Installer):
    """
    General form of a package manager :class:`Installer`
    implementation that assumes:

     - installer rosdep args spec is a list of package names stored with the key "packages"
     - a detect function exists that can return a list of packages that are installed

    Also, if *supports_depends* is set to ``True``:
    
     - installer rosdep args spec can also include dependency specification with the key "depends"
    """

    def __init__(self, detect_fn, supports_depends=False):
        """
        :param supports_depends: package manager supports dependency key
        """
        self.detect_fn = detect_fn
        self.supports_depends = supports_depends
        self.as_root = True
        self.sudo_command = 'sudo -H'

    def elevate_priv(self, cmd):
        """
        Prepend *self.sudo_command* to the command if *self.as_root* is ``True``.

        :param list cmd: list of strings comprising the command
        :returns: a list of commands
        """
        return (self.sudo_command.split() if self.as_root else []) + cmd

    def resolve(self, rosdep_args):
        """
        See :meth:`Installer.resolve()`
        """
        packages = None
        if type(rosdep_args) == dict:
            packages = rosdep_args.get("packages", [])
            if type(packages) == type("string"):
                packages = packages.split()
        elif type(rosdep_args) == type('str'):
            packages = rosdep_args.split(' ')
        elif type(rosdep_args) == list:
            packages = rosdep_args
        else:
            raise InvalidData("Invalid rosdep args: %s"%(rosdep_args))
        return packages

    def unique(self, *resolved_rules):
        """
        See :meth:`Installer.unique()`
        """
        s = set()
        for resolved in resolved_rules:
            s.update(resolved)
        return sorted(list(s))
        
    def get_packages_to_install(self, resolved, reinstall=False):
        if reinstall:
            return resolved
        if not resolved:
            return []
        else:
            return list(set(resolved) - set(self.detect_fn(resolved)))

    def is_installed(self, resolved_item):
        return not self.get_packages_to_install([resolved_item])

    def get_install_command(self, resolved, interactive=True, reinstall=False, quiet=False):
        raise NotImplementedError('subclasses must implement', resolved, interactive, reinstall, quiet)

    def get_depends(self, rosdep_args): 
        """ 
        :returns: list of dependencies on other rosdep keys.  Only
          necessary if the package manager doesn't handle
          dependencies.
        """
        if self.supports_depends and type(rosdep_args) == dict:
            return rosdep_args.get('depends', [])
        return [] # Default return empty list

class RosdepInstaller(object):

    def __init__(self, installer_context, lookup):
        self.installer_context = installer_context
        self.lookup = lookup
        
    def get_uninstalled(self, resources, implicit=False, verbose=False):
        """
        Get list of system dependencies that have not been installed
        as well as a list of errors from performing the resolution.
        This is a bulk API in order to provide performance
        optimizations in checking install state.

        :param resources: List of resource names (e.g. ROS package names), ``[str]]``
        :param implicit: Install implicit (recursive) dependencies of
            resources.  Default ``False``.

        :returns: (uninstalled, errors), ``({str: [opaque]}, {str: ResolutionError})``.
          Uninstalled is a dictionary with the installer_key as the key.
        :raises: :exc:`RosdepInternalError`
        """
        
        installer_context = self.installer_context

        # resolutions have been unique()d
        if verbose:
            print("resolving for resources [%s]"%(', '.join(resources)))
        resolutions, errors = self.lookup.resolve_all(resources, installer_context, implicit=implicit)

        # for each installer, figure out what is left to install
        uninstalled = []
        if resolutions == []:
            return uninstalled, errors
        for installer_key, resolved in resolutions: #py3k
            if verbose:
                print("resolution: %s [%s]" % (installer_key, ', '.join([str(r) for r in resolved])))
            try:
                installer = installer_context.get_installer(installer_key)
            except KeyError as e: # lookup has to be buggy to cause this
                raise RosdepInternalError(e)
            try:
                packages_to_install = installer.get_packages_to_install(resolved)
            except Exception as e:
                rd_debug(traceback.format_exc())
                raise RosdepInternalError(e, message="Bad installer [%s]: %s"%(installer_key, e))

            # only create key if there is something to do
            if packages_to_install:
                uninstalled.append((installer_key, packages_to_install))
            if verbose:
                print("uninstalled: [%s]"%(', '.join([str(p) for p in packages_to_install])))
        
        return uninstalled, errors
    
    def install(self, uninstalled, interactive=True, simulate=False,
                continue_on_error=False, reinstall=False, verbose=False, quiet=False):
        """
        Install the uninstalled rosdeps.  This API is for the bulk
        workflow of rosdep (see example below).  For a more targeted
        install API, see :meth:`RosdepInstaller.install_resolved`.

        :param uninstalled: uninstalled value from
          :meth:`RosdepInstaller.get_uninstalled`.  Value is a
          dictionary mapping installer key to a dictionary with resolution
          data, ``{str: {str: vals}}``
        :param interactive: If ``False``, suppress
          interactive prompts (e.g. by passing '-y' to ``apt``).
        :param simulate: If ``False`` simulate installation
          without actually executing.
        :param continue_on_error: If ``True``, continue installation
          even if an install fails.  Otherwise, stop after first
          installation failure.
        :param reinstall: If ``True``, install dependencies if even
          already installed (default ``False``).

        :raises: :exc:`InstallFailed` if any rosdeps fail to install
          and *continue_on_error* is ``False``.
        :raises: :exc:`KeyError` If *uninstalled* value has invalid
          installer keys
        
        Example::

            uninstalled, errors = installer.get_uninstalled(packages)
            installer.install(uninstalled)
        """
        if verbose:
            print("install options: reinstall[%s] simulate[%s] interactive[%s]"%(reinstall, simulate, interactive))
            print("install: uninstalled keys are %s"%(', '.join([', '.join(pkg) for pkg in [v for k,v in uninstalled]])))

        # Squash uninstalled again, in case some dependencies were already installed
        squashed_uninstalled = []
        previous_installer_key = None
        for installer_key, resolved in uninstalled:
            if previous_installer_key != installer_key:
                squashed_uninstalled.append((installer_key, []))
                previous_installer_key = installer_key
            squashed_uninstalled[-1][1].extend(resolved)

        failures = []
        for installer_key, resolved in squashed_uninstalled:
            try:
                self.install_resolved(installer_key, resolved, simulate=simulate,
                                      interactive=interactive, reinstall=reinstall, continue_on_error=continue_on_error,
                                      verbose=verbose, quiet=quiet)
            except InstallFailed as e:
                if not continue_on_error:
                    raise
                else:
                    #accumulate errors
                    failures.extend(e.failures)
        if failures:
            raise InstallFailed(failures=failures)

    def install_resolved(self, installer_key, resolved, simulate=False, interactive=True,
                         reinstall=False, continue_on_error=False, verbose=False, quiet=False):
        """
        Lower-level API for installing a rosdep dependency.  The
        rosdep keys have already been resolved to *installer_key* and
        *resolved* via :exc:`RosdepLookup` or other means.
        
        :param installer_key: Key for installer to apply to *resolved*, ``str``
        :param resolved: Opaque resolution list from :class:`RosdepLookup`.
        :param interactive: If ``True``, allow interactive prompts (default ``True``)
        :param simulate: If ``True``, don't execute installation commands, just print to screen.
        :param reinstall: If ``True``, install dependencies if even
          already installed (default ``False``).
        :param verbose: If ``True``, print verbose output to screen (default ``False``)
        :param quiet: If ``True``, supress output except for errors (default ``False``)
        
        :raises: :exc:`InstallFailed` if any of *resolved* fail to install.
        """
        installer_context = self.installer_context
        installer = installer_context.get_installer(installer_key)
        command = installer.get_install_command(resolved, interactive=interactive, reinstall=reinstall, quiet=quiet)
        if not command:
            if verbose:
                print("#No packages to install")
            return

        if simulate:
            print("#[%s] Installation commands:"%(installer_key))
            for sub_command in command:
                print('  '+' '.join(sub_command))

        # nothing left to do for simulation
        if simulate:
            return
        
        # run each install command set and collect errors
        failures = []
        for sub_command in command:
            # always echo commands to screen
            print_bold("executing command [%s]"%' '.join(sub_command))
            result = subprocess.call(sub_command)
            if verbose:
                print("command return code [%s]: %s"%(' '.join(sub_command), result))
            if result != 0:
                failures.append((installer_key, 'command [%s] failed'%(' '.join(sub_command))) )
                if not continue_on_error:
                    raise InstallFailed(failures=failures)

        # test installation of each
        for r in resolved:
            if not installer.is_installed(r):
                failures.append((installer_key, "Failed to detect successful installation of [%s]"%(r)))
        # finalize result
        if failures:
            raise InstallFailed(failures=failures)
        elif verbose:
            print("#successfully installed")
