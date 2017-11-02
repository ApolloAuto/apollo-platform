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

from .package import Package
from .release_repository_specification import ReleaseRepositorySpecification


class ReleaseFile(object):

    _type = 'distribution'

    def __init__(self, name, data):
        self.name = name

        assert 'type' in data and data['type'] != 'release', "Unable to handle 'release' format anymore, please update your 'release' file to the latest specification"
        assert 'type' in data, "Expected file type is '%s'" % ReleaseFile._type
        assert data['type'] == ReleaseFile._type, "Expected file type is '%s', not '%s'" % (ReleaseFile._type, data['type'])

        assert 'version' in data, "Release file for '%s' lacks required version information" % self.name
        assert int(data['version']) == 1, "Unable to handle '%s' format version '%d', please update rosdistro (e.g. on Ubuntu/Debian use: sudo apt-get update && sudo apt-get install --only-upgrade python-rosdistro)" % (ReleaseFile._type, int(data['version']))
        self.version = int(data['version'])

        self.repositories = {}
        self.packages = {}
        if 'repositories' in data:
            for repo_name in sorted(data['repositories'].keys()):
                if 'release' not in data['repositories'][repo_name]:
                    continue
                repo_status_data = data['repositories'][repo_name]
                repo_data = repo_status_data['release']
                try:
                    repo = ReleaseRepositorySpecification(repo_name, repo_data)
                    # for backward compatibility only
                    repo.status = repo_status_data.get('status', None)
                    repo.status_description = repo_status_data.get('status_description', None)
                except AssertionError as e:
                    e.args = [("Release file '%s': %s" % (self.name, a) if i == 0 else a) for i, a in enumerate(e.args)]
                    raise e
                self.repositories[repo_name] = repo

                if repo.package_names:
                    for pkg_name in repo.package_names:
                        assert pkg_name not in self.packages, "Duplicate package name '%s' exists in repository '%s' as well as in repository '%s'" % (pkg_name, repo_name, self.packages[pkg_name].repository_name)
                        self._add_package(pkg_name, repo, unary_repo=len(repo.package_names) == 1 and pkg_name == repo_name)
                else:
                    # no package means a single package in the root of the repository
                    self._add_package(repo_name, repo, {'subfolder': '.'}, True)

        self.platforms = {}
        if 'release_platforms' in data:
            for os_name in data['release_platforms'].keys():
                self.platforms[os_name] = []
                for os_code_name in data['release_platforms'][os_name]:
                    assert os_code_name not in self.platforms[os_name], "Distribution '%s' specifies the os_code_name '%s' multiple times for the os_name '%s'" % (self.name, os_code_name, os_name)
                    self.platforms[os_name].append(os_code_name)

    def _add_package(self, pkg_name, repo, unary_repo):
        assert pkg_name not in self.packages
        self.packages[pkg_name] = Package(pkg_name, repo.name)
