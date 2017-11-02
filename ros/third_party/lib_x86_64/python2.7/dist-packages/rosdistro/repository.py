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

from .doc_repository_specification import DocRepositorySpecification
from .release_repository_specification import ReleaseRepositorySpecification
from .source_repository_specification import SourceRepositorySpecification
from .status import valid_statuses


class Repository(object):

    def __init__(self, name, doc_data, release_data, source_data, status_data):
        self.name = name

        self.doc_repository = DocRepositorySpecification(self.name, doc_data) if doc_data else None
        self.release_repository = ReleaseRepositorySpecification(self.name, release_data) if release_data else None
        self.source_repository = SourceRepositorySpecification(self.name, source_data) if source_data else None

        self.status = status_data.get('status', None)
        if self.status is not None:
            assert self.status in valid_statuses
        self.status_description = status_data.get('status_description', None)

        self.status_per_package = {}
        for pkg_name in status_data.get('status_per_package', {}):
            data = {}
            status = status_data.get('status_per_package')[pkg_name].get('status', None)
            if status:
                assert status in valid_statuses
                data['status'] = status
            status_description = status_data.get('status_per_package')[pkg_name].get('status_description', None)
            if status_description:
                data['status_description'] = status_description
            if data:
                self.status_per_package[pkg_name] = data

    def get_data(self):
        data = {}

        if self.doc_repository:
            data['doc'] = self.doc_repository.get_data()
        if self.release_repository:
            data['release'] = self.release_repository.get_data()
        if self.source_repository:
            data['source'] = self.source_repository.get_data()

        if self.status:
            data['status'] = self.status
        if self.status_description:
            data['status_description'] = self.status_description

        if self.status_per_package:
            data['status_per_package'] = {}
            for pkg_name in sorted(self.status_per_package.keys()):
                pkg_status = self.status_per_package[pkg_name]
                pkg_data = {}
                status = pkg_status.get('status', None)
                if status:
                    pkg_data['status'] = status
                status_description = pkg_status.get('status_description', None)
                if status_description:
                    pkg_data['status_description'] = status_description
                if pkg_data:
                    data['status_per_package'][pkg_name] = pkg_data

        return data
