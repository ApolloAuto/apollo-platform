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


class ReleaseBuild(object):

    def __init__(self, dist, release_build_file):
        self._dist = dist
        self._build_file = release_build_file

        self._verify_package_names(self._build_file.package_whitelist)
        self._verify_package_names(self._build_file.package_blacklist)

        for os_name in self._build_file.get_target_os_names():
            assert os_name in self._dist.platforms.keys(), "Distribution '%s' specifies to build for os_name '%s' which is not listed in the release file" % (self._dist.name, os_name)
            for os_code_name in self._build_file.get_target_os_code_names(os_name):
                assert os_code_name in self._dist.platforms[os_name], "Distribution '%s' specifies to build for os_code_name '%s' which is not listed in the release file" % (self._dist.name, os_code_name)

        self._verify_package_names(self._build_file.sync_packages)

    def _verify_package_names(self, pkg_names):
        if pkg_names:
            for pkg_name in pkg_names:
                assert pkg_name in self._dist.packages, "Distribution '%s' specifies to build the package '%s' which is not listed in the release file" % (self._dist.name, pkg_name)

    def __getattr__(self, name):
        return getattr(self._build_file, name)

    def get_package_names(self):
        if not self._build_file.package_whitelist and not self._build_file.package_blacklist:
            return self._dist.get_package_names()
        raise NotImplementedError('Package whitelisting/blacklisting has not been implemented yet')
