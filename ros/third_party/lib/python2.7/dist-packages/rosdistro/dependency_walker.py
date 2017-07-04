# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Open Source Robotics Foundation, Inc. nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from catkin_pkg.package import InvalidPackage, parse_package_string


class DependencyWalker(object):

    def __init__(self, distribution_instance):
        self._distribution_instance = distribution_instance
        self._packages = {}

    def _get_package(self, pkg_name):
        if pkg_name not in self._packages:
            repo = self._distribution_instance.repositories[self._distribution_instance.release_packages[pkg_name].repository_name].release_repository
            assert repo is not None and repo.version is not None, "Package '%s' in repository '%s' has no version set" % (pkg_name, repo.name)
            assert 'release' in repo.tags, "Package '%s' in repository '%s' has no 'release' tag set" % (pkg_name, repo.name)
            pkg_xml = self._distribution_instance.get_release_package_xml(pkg_name)
            try:
                pkg = parse_package_string(pkg_xml)
            except InvalidPackage as e:
                raise InvalidPackage(pkg_name + ': %s' % str(e))
            self._packages[pkg_name] = pkg
        return self._packages[pkg_name]

    def get_depends(self, pkg_name, depend_type, ros_packages_only=False):
        '''Return a set of package names which the package depends on.'''
        deps = self._get_dependencies(pkg_name, depend_type)
        if ros_packages_only:
            deps &= set(self._distribution_instance.release_packages.keys())
        return deps

    def get_recursive_depends(self, pkg_name, depend_types, ros_packages_only=False, ignore_pkgs=None, limit_depth=None):
        '''Return a set of package names which the package (transitively) depends on.'''
        ignore_pkgs = set(ignore_pkgs or [])
        depends = set([])
        # mapping from the scheduled pkg names to their dependency level
        pkgs_to_check = {pkg_name: 0}
        while pkgs_to_check:
            next_pkg_to_check = sorted(pkgs_to_check.keys())[0]
            current_level = pkgs_to_check.pop(next_pkg_to_check)
            if next_pkg_to_check in ignore_pkgs or (limit_depth is not None and current_level >= limit_depth):
                continue
            for depend_type in depend_types:
                deps = self.get_depends(next_pkg_to_check, depend_type, ros_packages_only=ros_packages_only)
                deps -= ignore_pkgs
                new_deps = deps - depends
                for new_dep in new_deps:
                    if new_dep not in pkgs_to_check:
                        pkgs_to_check[new_dep] = current_level + 1
                    else:
                        pkgs_to_check[new_dep] = min(pkgs_to_check[new_dep], current_level + 1)
                depends |= new_deps
        return depends

    def get_depends_on(self, pkg_name, depend_type, ignore_pkgs=None):
        '''Return a set of package names which depend on the package.'''
        ignore_pkgs = ignore_pkgs or []
        depends_on = set([])
        for name in self._distribution_instance.release_packages.keys():
            if name in ignore_pkgs:
                continue
            pkg = self._distribution_instance.release_packages[name]
            repo = self._distribution_instance.repositories[pkg.repository_name].release_repository
            if repo is None or repo.version is None:
                continue
            deps = self._get_dependencies(name, depend_type)
            if pkg_name in deps:
                depends_on.add(name)
        return depends_on

    def get_recursive_depends_on(self, pkg_name, depend_types, ignore_pkgs=None):
        '''Return a set of package names which (transitively) depend on the package.'''
        ignore_pkgs = ignore_pkgs or []
        depends_on = set([])
        pkgs_to_check = set([pkg_name])
        while pkgs_to_check:
            next_pkg_to_check = pkgs_to_check.pop()
            for depend_type in depend_types:
                deps = self.get_depends_on(next_pkg_to_check, depend_type, ignore_pkgs=ignore_pkgs)
                new_deps = deps - depends_on
                pkgs_to_check |= new_deps
                depends_on |= new_deps
        return depends_on

    def _get_dependencies(self, pkg_name, dep_type):
        pkg = self._get_package(pkg_name)
        deps = {
            'buildtool': pkg.buildtool_depends,
            'build': pkg.build_depends,
            'run': pkg.run_depends,
            'test': pkg.test_depends
        }
        return set([d.name for d in deps[dep_type]])
