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


class ReleaseBuildFile(object):

    _type = 'release-build'

    def __init__(self, name, data):
        self.name = name

        assert 'type' in data, "Expected file type is '%s'" % ReleaseBuildFile._type
        assert data['type'] == ReleaseBuildFile._type, "Expected file type is '%s', not '%s'" % (ReleaseBuildFile._type, data['type'])

        assert 'version' in data, "Release build file for '%s' lacks required version information" % self.name
        assert int(data['version']) == 1, "Unable to handle '%s' format version '%d', please update rosdistro (e.g. on Ubuntu/Debian use: sudo apt-get update && sudo apt-get install --only-upgrade python-rosdistro)" % (ReleaseBuildFile._type, int(data['version']))
        self.version = int(data['version'])

        self.package_whitelist = []
        if 'package_whitelist' in data:
            self.package_whitelist = data['package_whitelist']
            assert isinstance(self.package_whitelist, list)
        self.package_blacklist = []
        if 'package_blacklist' in data:
            self.package_blacklist = data['package_blacklist']
            assert isinstance(self.package_blacklist, list)

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
        self.jenkins_sourcedeb_job_timeout = None
        if 'jenkins_sourcedeb_job_timeout' in data:
            self.jenkins_sourcedeb_job_timeout = int(data['jenkins_sourcedeb_job_timeout'])
        self.jenkins_binarydeb_job_timeout = None
        if 'jenkins_binarydeb_job_timeout' in data:
            self.jenkins_binarydeb_job_timeout = int(data['jenkins_binarydeb_job_timeout'])

        self.sync_package_count = None
        self.sync_packages = []
        if 'sync' in data:
            if 'package_count' in data['sync']:
                self.sync_package_count = int(data['sync']['package_count'])
            if 'packages' in data['sync']:
                self.notify_maintainers = data['sync']['packages']
                assert isinstance(self.sync_packages, list)

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
        config = {}
        if '_config' in self._targets:
            config.update(self._targets['_config'])
        if os_name is not None:
            if '_config' in self._targets[os_name]:
                config.update(self._targets[os_name]['_config'])
            if os_code_name is not None:
                if '_config' in self._targets[os_name][os_code_name]:
                    config.update(self._targets[os_name][os_code_name]['_config'])
                if arch is not None:
                    if '_config' in self._targets[os_name][os_code_name][arch]:
                        config.update(self._targets[os_name][os_code_name][arch]['_config'])
        return config

    def get_data(self):
        data = {}
        data['type'] = ReleaseBuildFile._type
        data['version'] = 1
        if self.package_whitelist:
            data['package_whitelist'] = self.package_whitelist
        if self.package_blacklist:
            data['package_blacklist'] = self.package_blacklist

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
        if self.jenkins_sourcedeb_job_timeout:
            data['jenkins_sourcedeb_job_timeout'] = self.jenkins_sourcedeb_job_timeout
        if self.jenkins_binarydeb_job_timeout:
            data['jenkins_binarydeb_job_timeout'] = self.jenkins_binarydeb_job_timeout

        if self.sync_package_count or self.sync_packages:
            data['sync'] = {}
            if self.sync_package_count is not None:
                data['sync']['package_count'] = self.sync_package_count
            if self.sync_packages:
                data['sync']['packages'] = self.sync_packages

        return data
