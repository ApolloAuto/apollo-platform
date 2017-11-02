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

import copy


class SourceBuildFile(object):

    _type = 'source-build'

    def __init__(self, name, data):
        self.name = name

        assert 'type' in data, "Expected file type is '%s'" % SourceBuildFile._type
        assert data['type'] == SourceBuildFile._type, "Expected file type is '%s', not '%s'" % (SourceBuildFile._type, data['type'])

        assert 'version' in data, "Source build file for '%s' lacks required version information" % self.name
        assert int(data['version']) == 1, "Unable to handle '%s' format version '%d', please update rosdistro (e.g. on Ubuntu/Debian use: sudo apt-get update && sudo apt-get install --only-upgrade python-rosdistro)" % (SourceBuildFile._type, int(data['version']))
        self.version = int(data['version'])

        self.repository_whitelist = []
        if 'repository_whitelist' in data:
            self.repository_whitelist = data['repository_whitelist']
            assert isinstance(self.repository_whitelist, list)
        self.repository_blacklist = []
        if 'repository_blacklist' in data:
            self.repository_blacklist = data['repository_blacklist']
            assert isinstance(self.repository_blacklist, list)

        self.notify_emails = []
        self.notify_maintainers = None
        self.notify_committers = None
        if 'notifications' in data:
            if 'emails' in data['notifications']:
                self.notify_emails = data['notifications']['emails']
                assert isinstance(self.notify_emails, list)
            if 'maintainers' in data['notifications'] and data['notifications']['maintainers']:
                self.notify_maintainers = True
            if 'committers' in data['notifications'] and data['notifications']['committers']:
                self.notify_committers = True

        assert 'targets' in data
        self._targets = {}
        for os_name in data['targets'].keys():
            if os_name == '_config':
                self._targets[os_name] = data['targets'][os_name]
                continue
            self._targets[os_name] = {}
            for os_code_name in data['targets'][os_name]:
                if os_code_name == '_config':
                    self._targets[os_name][os_code_name] = data['targets'][os_name][os_code_name]
                    continue
                self._targets[os_name][os_code_name] = {}
                for arch in data['targets'][os_name][os_code_name]:
                    self._targets[os_name][os_code_name][arch] = data['targets'][os_name][os_code_name][arch]

        assert 'jenkins_url' in data
        self.jenkins_url = str(data['jenkins_url'])
        self.jenkins_job_timeout = None
        if 'jenkins_job_timeout' in data:
            self.jenkins_job_timeout = int(data['jenkins_job_timeout'])

    def filter_repositories(self, repos):
        res = copy.copy(set(repos))
        if self.repository_whitelist:
            res &= set(self.repository_whitelist)
        res -= set(self.repository_blacklist)
        return res

    def get_target_os_names(self):
        return [t for t in self._targets.keys() if t != '_config']

    def get_target_os_code_names(self, os_name):
        os_code_names = self._targets[os_name]
        return [t for t in os_code_names.keys() if t != '_config']

    def get_target_arches(self, os_name, os_code_name):
        arches = self._targets[os_name][os_code_name]
        return [t for t in arches.keys() if t != '_config']

    def get_target_configuration(self, os_name=None, os_code_name=None, arch=None):
        assert os_code_name is not None or arch is None
        assert os_name is not None or os_code_name is None
        arches = self._targets[os_name][os_code_name]
        return [t for t in arches.keys() if t != '_config']

    def get_data(self):
        data = {}
        data['type'] = SourceBuildFile._type
        data['version'] = 1
        if self.repository_whitelist:
            data['repository_whitelist'] = self.repository_whitelist
        if self.repository_blacklist:
            data['repository_blacklist'] = self.repository_blacklist

        if self.notify_emails or self.notify_maintainers or self.notify_committers:
            data['notifications'] = {}
            if self.notify_emails:
                data['notifications']['emails'] = self.notify_emails
            if self.notify_maintainers is not None:
                data['notifications']['maintainers'] = bool(self.notify_maintainers)
            if self.notify_committers is not None:
                data['notifications']['committers'] = bool(self.notify_committers)

        data['targets'] = self._targets

        data['jenkins_url'] = self.jenkins_url
        if self.jenkins_job_timeout:
            data['jenkins_job_timeout'] = self.jenkins_job_timeout

        return data
