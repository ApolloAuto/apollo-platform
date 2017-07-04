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

# legacy imports
from . import common
from .rosdistro import walks
from .rosdistro import RosDistro
from .develdistro import DevelDistro
from .aptdistro import AptDistro

import gzip
import logging
import os
try:
    from cStringIO import StringIO
except ImportError:
    from io import BytesIO as StringIO
import sys
import yaml

logger = logging.getLogger('rosdistro')

from ._version import __version__
from .distribution import Distribution
from .distribution_cache import DistributionCache
from .distribution_file import create_distribution_file
from .distribution_file import DistributionFile
from .doc_build_file import DocBuildFile
from .doc_file import DocFile
from .external.appdirs import user_config_dir, site_config_dir
from .index import Index
from .loader import load_url
from .manifest_provider.cache import CachedManifestProvider
from .release import Release
from .release_build import ReleaseBuild
from .release_build_file import ReleaseBuildFile
from .release_cache import ReleaseCache
from .release_file import ReleaseFile
from .source_build_file import SourceBuildFile
from .source_file import SourceFile

### index information


DEFAULT_INDEX_URL = 'https://raw.githubusercontent.com/ros/rosdistro/master/index.yaml'


def get_index_url():
    # environment variable has precedence over configuration files
    if 'ROSDISTRO_INDEX_URL' in os.environ:
        return os.environ['ROSDISTRO_INDEX_URL']

    def read_cfg_index_url(fname):
        try:
            with open(fname) as f:
                return yaml.load(f.read())['index_url']
        except (IOError, KeyError, yaml.YAMLError):
            return None

    cfg_file = 'config.yaml'

    # first, look for the user configuration (usually ~/.config/rosdistro)
    user_cfg_path = os.path.join(user_config_dir('rosdistro'), cfg_file)
    index_url = read_cfg_index_url(user_cfg_path)
    if index_url is not None:
        return index_url

    # if not found, look for the global configuration *usually /etc/xdg/rosdistro)
    site_cfg_paths = os.path.join(site_config_dir('rosdistro', multipath=True), cfg_file).split(os.pathsep)
    for site_cfg_path in site_cfg_paths:
        index_url = read_cfg_index_url(site_cfg_path)
        if index_url is not None:
            return index_url

    # if nothing is found, use the default
    return DEFAULT_INDEX_URL


def get_index(url):
    logger.debug('Load index from "%s"' % url)
    yaml_str = load_url(url)
    data = yaml.load(yaml_str)
    base_url = os.path.dirname(url)
    return Index(data, base_url)


### distribution information


def get_distribution(index, dist_name):
    dist_file = get_distribution_file(index, dist_name)
    return Distribution(dist_file)


def get_distribution_file(index, dist_name):
    data = _get_dist_file_data(index, dist_name, 'distribution')
    return create_distribution_file(dist_name, data)


def get_distribution_files(index, dist_name):
    data = _get_dist_file_data(index, dist_name, 'distribution')
    if not isinstance(data, list):
        data = [data]
    dist_files = []
    for d in data:
        dist_file = DistributionFile(dist_name, d)
        dist_files.append(dist_file)
    return dist_files


def get_cached_distribution(index, dist_name, cache=None, allow_lazy_load=False):
    if cache is None:
        try:
            cache = get_distribution_cache(index, dist_name)
        except Exception:
            if not allow_lazy_load:
                raise
            # create empty cache instance
            dist_file_data = _get_dist_file_data(index, dist_name, 'distribution')
            cache = DistributionCache(dist_name, distribution_file_data=dist_file_data)
    dist = Distribution(
        cache.distribution_file,
        [CachedManifestProvider(cache, Distribution.default_manifest_providers if allow_lazy_load else None)])
    assert cache.distribution_file.name == dist_name
    return dist


def get_distribution_cache(index, dist_name):
    if dist_name not in index.distributions.keys():
        raise RuntimeError("Unknown distribution: '{0}'. Valid distribution names are: {1}".format(dist_name, ', '.join(["'%s'" % d for d in index.distributions.keys()])))
    dist = index.distributions[dist_name]
    if 'distribution_cache' not in dist.keys():
        raise RuntimeError("Distribution has no cache: '{0}'".format(dist_name))
    url = dist['distribution_cache']

    logger.debug('Load cache from "%s"' % url)
    if url.endswith('.yaml'):
        yaml_str = load_url(url)
    elif url.endswith('.yaml.gz'):
        yaml_gz_str = load_url(url, skip_decode=True)
        yaml_gz_stream = StringIO(yaml_gz_str)
        f = gzip.GzipFile(fileobj=yaml_gz_stream, mode='rb')
        yaml_str = f.read()
        if not isinstance(yaml_str, str):
            yaml_str = yaml_str.decode('utf-8')
        f.close()
    else:
        raise NotImplementedError('The url of the cache must end with either ".yaml" or ".yaml.gz"')
    data = yaml.load(yaml_str)
    return DistributionCache(dist_name, data)


### release information


def get_cached_release(index, dist_name, cache=None, allow_lazy_load=False):
    print('# rosdistro.get_cached_release() has been deprecated in favor of the new function rosdistro.get_cached_distribution() - please check that you have the latest versions of the Python tools (e.g. on Ubuntu/Debian use: sudo apt-get update && sudo apt-get install --only-upgrade python-bloom python-rosdep python-rosinstall python-rosinstall-generator)', file=sys.stderr)
    if cache is None:
        try:
            cache = get_release_cache(index, dist_name)
        except Exception:
            if not allow_lazy_load:
                raise
            # create empty cache instance
            rel_file_data = _get_dist_file_data(index, dist_name, 'release')
            cache = ReleaseCache(dist_name, rel_file_data=rel_file_data)
    rel = Release(
        cache.release_file,
        [CachedManifestProvider(cache, Release.default_manifest_providers if allow_lazy_load else None)])
    assert cache.release_file.name == dist_name
    return rel


def get_release(index, dist_name):
    print('# rosdistro.get_release() has been deprecated in favor of the new function rosdistro.get_cached_distribution() - please check that you have the latest versions of the Python tools (e.g. on Ubuntu/Debian use: sudo apt-get update && sudo apt-get install --only-upgrade python-bloom python-rosdep python-rosinstall python-rosinstall-generator)', file=sys.stderr)
    rel_file = get_release_file(index, dist_name)
    return Release(rel_file)


def get_release_file(index, dist_name):
    print('# rosdistro.get_release_file() has been deprecated in favor of the new function rosdistro.get_distribution_file() - please check that you have the latest versions of the Python tools (e.g. on Ubuntu/Debian use: sudo apt-get update && sudo apt-get install --only-upgrade python-bloom python-rosdep python-rosinstall python-rosinstall-generator)', file=sys.stderr)
    data = _get_dist_file_data(index, dist_name, 'distribution')
    return ReleaseFile(dist_name, data)


def get_release_cache(index, dist_name):
    print('# rosdistro.get_release_cache() has been deprecated in favor of the new function rosdistro.get_distribution_cache() - please check that you have the latest versions of the Python tools (e.g. on Ubuntu/Debian use: sudo apt-get update && sudo apt-get install --only-upgrade python-bloom python-rosdep python-rosinstall python-rosinstall-generator)', file=sys.stderr)
    if dist_name not in index.distributions.keys():
        raise RuntimeError("Unknown release: '{0}'. Valid release names are: {1}".format(dist_name, ', '.join(["'%s'" % d for d in index.distributions.keys()])))
    dist = index.distributions[dist_name]
    if 'distribution_cache' not in dist.keys():
        raise RuntimeError("Release has no cache: '{0}'".format(dist_name))
    url = dist['distribution_cache']

    logger.debug('Load cache from "%s"' % url)
    if url.endswith('.yaml'):
        yaml_str = load_url(url)
    elif url.endswith('.yaml.gz'):
        yaml_gz_str = load_url(url, skip_decode=True)
        yaml_gz_stream = StringIO(yaml_gz_str)
        f = gzip.GzipFile(fileobj=yaml_gz_stream, mode='rb')
        yaml_str = f.read()
        if not isinstance(yaml_str, str):
            yaml_str = yaml_str.decode('utf-8')
        f.close()
    else:
        raise NotImplementedError('The url of the cache must end with either ".yaml" or ".yaml.gz"')
    data = yaml.load(yaml_str)
    return ReleaseCache(dist_name, data)


def get_release_builds(index, release_file):
    print("# rosdistro.get_release_builds() has been deprecated and its functionality is now handled by the 'ros_buildfarm.config' module", file=sys.stderr)
    build_files = get_release_build_files(index, release_file.name)
    builds = []
    for build_file in build_files:
        builds.append(ReleaseBuild(release_file, build_file))
    return builds


def get_release_build_files(index, dist_name):
    print("# rosdistro.get_release_build_files() has been deprecated and its functionality is now handled by the 'ros_buildfarm.config' module", file=sys.stderr)
    data = _get_dist_file_data(index, dist_name, 'release_builds')
    build_files = []
    for d in data:
        build_files.append(ReleaseBuildFile(dist_name, d))
    return build_files


### source information


def get_source_file(index, dist_name):
    print('# rosdistro.get_source_file() has been deprecated in favor of the new function rosdistro.get_distribution_file() - please check that you have the latest versions of the Python tools (e.g. on Ubuntu/Debian use: sudo apt-get update && sudo apt-get install --only-upgrade python-bloom python-rosdep python-rosinstall python-rosinstall-generator)', file=sys.stderr)
    data = _get_dist_file_data(index, dist_name, 'distribution')
    return SourceFile(dist_name, data)


def get_source_build_files(index, dist_name):
    print("# rosdistro.get_source_build_files() has been deprecated and its functionality is now handled by the 'ros_buildfarm.config' module", file=sys.stderr)
    data = _get_dist_file_data(index, dist_name, 'source_builds')
    build_files = []
    for d in data:
        build_files.append(SourceBuildFile(dist_name, d))
    return build_files


### doc information


def get_doc_file(index, dist_name):
    print('# rosdistro.get_doc_file() has been deprecated in favor of the new function rosdistro.get_distribution_file() - please check that you have the latest versions of the Python tools (e.g. on Ubuntu/Debian use: sudo apt-get update && sudo apt-get install --only-upgrade python-bloom python-rosdep python-rosinstall python-rosinstall-generator)', file=sys.stderr)
    data = _get_dist_file_data(index, dist_name, 'distribution')
    return DocFile(dist_name, data)


def get_doc_build_files(index, dist_name):
    print("# rosdistro.get_doc_build_files() has been deprecated and its functionality is now handled by the 'ros_buildfarm.config' module", file=sys.stderr)
    data = _get_dist_file_data(index, dist_name, 'doc_builds')
    build_files = []
    for d in data:
        build_files.append(DocBuildFile(dist_name, d))
    return build_files


### internal


def _get_dist_file_data(index, dist_name, type_):
    if dist_name not in index.distributions.keys():
        raise RuntimeError("Unknown release: '{0}'. Valid release names are: {1}".format(dist_name, ', '.join(["'%s'" % d for d in index.distributions.keys()])))
    dist = index.distributions[dist_name]
    if type_ not in dist.keys():
        raise RuntimeError('unknown release type "%s"' % type_)
    url = dist[type_]

    def _load_yaml_data(url):
        logger.debug('Load file from "%s"' % url)
        yaml_str = load_url(url)
        return yaml.load(yaml_str)

    if not isinstance(url, list):
        data = _load_yaml_data(url)
    else:
        data = []
        for u in url:
            data.append(_load_yaml_data(u))
    return data
