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

from .manifest_provider.git import git_manifest_provider
from .manifest_provider.github import github_manifest_provider


class Release(object):

    default_manifest_providers = [github_manifest_provider, git_manifest_provider]

    def __init__(self, rel_file, manifest_providers=None):
        self._rel_file = rel_file
        # Use default
        self._manifest_providers = Release.default_manifest_providers
        # Override default if given
        if manifest_providers is not None:
            self._manifest_providers = manifest_providers
        self._package_xmls = {}

    def __getattr__(self, name):
        # for backward compatibility only
        if name == 'get_release_package_xml':
            return self.get_package_xml
        if name == 'release_packages':
            name = 'packages'
        if name == 'release_platforms':
            name = 'platforms'

        return getattr(self._rel_file, name)

    def get_package_xml(self, pkg_name):
        if pkg_name not in self._package_xmls:
            pkg = self._rel_file.packages[pkg_name]
            repo_name = pkg.repository_name
            repo = self._rel_file.repositories[repo_name]
            if repo.version is None:
                return None
            package_xml = None
            for mp in self._manifest_providers:
                #try:
                package_xml = mp(self._rel_file.name, repo, pkg_name)
                #except:
                #    pass
                if package_xml is not None:
                    break
            self._package_xmls[pkg_name] = package_xml
        return self._package_xmls[pkg_name]
