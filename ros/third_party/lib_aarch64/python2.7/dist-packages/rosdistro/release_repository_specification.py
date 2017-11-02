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

from .repository_specification import RepositorySpecification


class ReleaseRepositorySpecification(RepositorySpecification):

    def __init__(self, name, data):
        super(ReleaseRepositorySpecification, self).__init__(name, data)
        assert self.type == 'git'

        self.tags = {}
        if self.version is not None:
            assert 'tags' in data
            assert 'release' in data['tags']
        if 'tags' in data:
            for tag_type in data['tags']:
                tag_data = data['tags'][tag_type]
                self.tags[tag_type] = str(tag_data)

        self.package_names = []
        if 'packages' in data and data['packages']:
            self.package_names = sorted(data['packages'])
        else:
            # no packages means a single package
            self.package_names = [self.name]

        # for backward compatibility only
        self.release_repository = self

    def get_data(self):
        data = self._get_data(skip_git_type=True)
        if self.tags:
            data['tags'] = {}
            for tag in self.tags:
                data['tags'][tag] = str(self.tags[tag])
        if len(self.package_names) > 1 or (len(self.package_names) == 1 and self.package_names[0] != self.name):
            data['packages'] = sorted(self.package_names)
        return data
