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

import difflib
import sys
import yaml

from . import get_distribution_file, get_distribution_files, get_doc_build_files, get_doc_file, get_index, get_release_build_files, get_release_file, get_source_build_files, get_source_file
from .loader import load_url


def verify_files_parsable(index_url):
    return verify_files(index_url, _check_files_parsable, include_deprecated=True)


def _check_files_parsable(index, dist_name, loader_function, _yaml_url, _file_type):
    try:
        loader_function(index, dist_name)
    except Exception as e:
        print(str(e), file=sys.stderr)
        return False
    return True


def reformat_files(index_url):
    return verify_files(index_url, _reformat_files)


def verify_files_identical(index_url):
    return verify_files(index_url, _check_files_identical)


def verify_files(index_url, callback, include_deprecated=False):
    identical = True
    index = get_index(index_url)
    for dist_name in sorted(index.distributions.keys()):
        dist = index.distributions[dist_name]
        if index.version < 3:
            providers = [get_distribution_file]
        else:
            providers = [get_distribution_files]
        if include_deprecated:
            providers.extend([get_release_file, get_source_file, get_doc_file])
        file_providers = {
            'distribution': (providers, 'distribution'),
            'release_builds': (get_release_build_files, 'release-build'),
            'source_builds': (get_source_build_files, 'source-build'),
            'doc_builds': (get_doc_build_files, 'doc-build')
        }
        for key in sorted(file_providers.keys()):
            provider, file_type = file_providers[key]
            yaml_url = dist[key]
            if isinstance(provider, list):
                for p in provider:
                    identical &= callback(index, dist_name, p, yaml_url, file_type)
            else:
                identical &= callback(index, dist_name, provider, yaml_url, file_type)

    return identical


def _reformat_files(index, dist_name, loader_function, yaml_url, file_type):
    files = loader_function(index, dist_name)
    if not isinstance(files, list):
        files = [files]
        yaml_url = [yaml_url]
    for i, f in enumerate(files):
        url = yaml_url[i]
        if not url.startswith('file://'):
            print('Skipping non-file url: %s' % url)
            continue
        identical = _check_file_identical(f, yaml_url[i], file_type)
        path = url[7:]
        if identical:
            print('Skipping identical file: %s' % path)
            continue
        print('Updating file: %s' % path)
        data = f.get_data()
        dist_file_data = _to_yaml(data)
        dist_file_data = '\n'.join(_yaml_header_lines(file_type, data['version'])) + '\n' + dist_file_data
        with open(path, 'w') as f:
            f.write(dist_file_data)
    return True


def _check_files_identical(index, dist_name, loader_function, yaml_url, file_type):
    identical = True
    files = loader_function(index, dist_name)
    if not isinstance(files, list):
        files = [files]
        yaml_url = [yaml_url]
    for i, f in enumerate(files):
        identical &= _check_file_identical(f, yaml_url[i], file_type)
    return identical


def _check_file_identical(dist_file, yaml_url, file_type):
    yaml_str = load_url(yaml_url)
    yaml_lines = yaml_str.splitlines()
    dist_file_data = dist_file.get_data()
    dist_file_lines = _to_yaml(dist_file_data).splitlines()
    dist_file_lines[0:0] = _yaml_header_lines(file_type, dist_file_data['version'])

    if yaml_lines != dist_file_lines:
        diff = difflib.unified_diff(
            yaml_lines, dist_file_lines,
            yaml_url, 'loaded-and-saved',
            n=1, lineterm='')
        for line in diff:
            print(line, file=sys.stderr)
        return False
    return True


def _to_yaml(data):
    yaml_str = yaml.dump(data, default_flow_style=False)
    yaml_str = yaml_str.replace(': null', ':')
    yaml_str = yaml_str.replace(': {}', ':')
    return yaml_str


def _yaml_header_lines(file_type, version):
    rep = '141'
    if file_type == 'index' and version == 3:
        rep = '143'
    return [
        '%YAML 1.1',
        '# ROS %s file' % file_type,
        '# see REP %s: http://ros.org/reps/rep-0%s.html' % (rep, rep),
        '---'
    ]
