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

from __future__ import print_function

from .distribution_file import create_distribution_file


class DistributionCache(object):

    _type = 'cache'

    def __init__(self, name, data=None, distribution_file_data=None):
        assert data or distribution_file_data
        if data:
            assert 'type' in data, "Expected file type is '%s'" % DistributionCache._type
            assert data['type'] == DistributionCache._type, "Expected file type is '%s', not '%s'" % (DistributionCache._type, data['type'])

            assert 'version' in data, "Distribution cache file for '%s' lacks required version information" % name
            self.version = int(data['version'])
            assert self.version > 1, "Unable to handle '%s' format version '%d' anymore, please update your '%s' file to version '2'" % (DistributionCache._type, self.version, DistributionCache._type)
            assert self.version == 2, "Unable to handle '%s' format version '%d', please update rosdistro (e.g. on Ubuntu/Debian use: sudo apt-get update && sudo apt-get install --only-upgrade python-rosdistro)" % (DistributionCache._type, self.version)

            assert 'name' in data, "Distribution cache file for '%s' lacks required name information" % name
            assert data['name'] == name, "Distribution cache file for '%s' does not match the name '%s'" % (name, data['name'])
        else:
            self.version = 2

        self._distribution_file_data = data['distribution_file'] if data else distribution_file_data
        self.distribution_file = create_distribution_file(name, self._distribution_file_data)
        self.release_package_xmls = data['release_package_xmls'] if data else {}

        # if Python 2 has converted the xml to unicode, convert it back
        for k, v in self.release_package_xmls.items():
            if not isinstance(v, str) and not isinstance(v, bytes):
                self.release_package_xmls[k] = v.encode('utf-8')

    def get_data(self):
        data = {}
        data['type'] = 'cache'
        data['version'] = 2
        data['name'] = self.distribution_file.name
        data['distribution_file'] = self._distribution_file_data
        data['release_package_xmls'] = self.release_package_xmls
        return data

    def update_distribution(self, distribution_file_data):
        # remove packages which are not in the old distribution file
        self._remove_obsolete_entries()

        self._distribution_file_data = distribution_file_data
        dist_file = create_distribution_file(self.distribution_file.name, self._distribution_file_data)

        # remove all package xmls if repository information has changed
        for pkg_name in sorted(dist_file.release_packages.keys()):
            if pkg_name not in self.distribution_file.release_packages:
                continue
            if pkg_name in self.release_package_xmls and self._get_repo_info(dist_file, pkg_name) != self._get_repo_info(self.distribution_file, pkg_name):
                del self.release_package_xmls[pkg_name]

        self.distribution_file = dist_file
        # remove packages which are not in the new distribution file
        self._remove_obsolete_entries()

    def _get_repo_info(self, dist_file, pkg_name):
        pkg = dist_file.release_packages[pkg_name]
        repo = dist_file.repositories[pkg.repository_name].release_repository
        return (repo.version, repo.url)

    def _remove_obsolete_entries(self):
        for pkg_name in list(self.release_package_xmls.keys()):
            if pkg_name not in self.distribution_file.release_packages:
                print('- REMOVE', pkg_name)
                del self.release_package_xmls[pkg_name]
