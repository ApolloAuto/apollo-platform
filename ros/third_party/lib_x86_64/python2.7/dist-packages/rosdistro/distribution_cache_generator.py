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

import gzip
import os
import re
import sys
import yaml

from catkin_pkg.package import InvalidPackage, parse_package_string

from . import _get_dist_file_data, get_cached_distribution, get_index, get_distribution_cache
from .distribution_cache import DistributionCache


def generate_distribution_caches(index, dist_names=None, preclean=False, ignore_local=False, debug=False):
    if os.path.isfile(index):
        index = 'file://' + os.path.abspath(index)
    index = get_index(index)

    if not dist_names:
        dist_names = sorted(index.distributions.keys())

    errors = []
    caches = {}
    for dist_name in dist_names:
        try:
            cache = generate_distribution_cache(index, dist_name, preclean=preclean, ignore_local=ignore_local, debug=debug)
        except RuntimeError as e:
            errors.append(str(e))
            continue
        caches[dist_name] = cache
    if errors:
        raise RuntimeError('\n'.join(errors))
    return caches


def generate_distribution_cache(index, dist_name, preclean=False, ignore_local=False, debug=False):
    dist, cache = _get_cached_distribution(index, dist_name, preclean=preclean, ignore_local=ignore_local)
    # fetch all manifests
    print('- fetch missing manifests')
    errors = []
    for pkg_name in sorted(dist.release_packages.keys()):
        repo = dist.repositories[dist.release_packages[pkg_name].repository_name].release_repository
        if repo.version is None:
            if debug:
                print('  - skip "%s" since it has no version' % pkg_name)
            continue
        if debug:
            print('  - fetch "%s"' % pkg_name)
        else:
            sys.stdout.write('.')
            sys.stdout.flush()
        # check that package.xml is fetchable
        package_xml = dist.get_release_package_xml(pkg_name)
        if not package_xml:
            errors.append('%s: missing package.xml file for package "%s"' % (dist_name, pkg_name))
            continue
        # check that package.xml is parseable
        try:
            pkg = parse_package_string(package_xml)
        except InvalidPackage as e:
            errors.append('%s: invalid package.xml file for package "%s": %s' % (dist_name, pkg_name, e))
            continue
        # check that version numbers match (at least without deb inc)
        if not re.match('^%s-\d+$' % re.escape(pkg.version), repo.version):
            errors.append('%s: different version in package.xml (%s) for package "%s" than for the repository (%s) (after removing the debian increment)' % (dist_name, pkg.version, pkg_name, repo.version))

    if not debug:
        print('')

    if errors:
        raise RuntimeError('\n'.join(errors))

    return cache


def _get_cached_distribution(index, dist_name, preclean=False, ignore_local=False):
    print('Build cache for "%s"' % dist_name)
    cache = None
    try:
        if not preclean:
            if not ignore_local:
                print('- trying to use local cache')
                yaml_str = None
                if os.path.exists('%s-cache.yaml.gz' % dist_name):
                    print('- use local file "%s-cache.yaml.gz"' % dist_name)
                    with gzip.open('%s-cache.yaml.gz' % dist_name, 'rb') as f:
                        yaml_str = f.read()
                elif os.path.exists('%s-cache.yaml' % dist_name):
                    print('- use local file "%s-cache.yaml"' % dist_name)
                    with open('%s-cache.yaml' % dist_name, 'r') as f:
                        yaml_str = f.read()
                if yaml_str is not None:
                    data = yaml.load(yaml_str)
                    cache = DistributionCache(dist_name, data)
            if not cache:
                print('- trying to fetch cache')
                # get distribution cache
                cache = get_distribution_cache(index, dist_name)
            # get current distribution file
            rel_file_data = _get_dist_file_data(index, dist_name, 'distribution')
            # update cache with current distribution file
            cache.update_distribution(rel_file_data)
    except Exception as e:
        print('- failed to fetch old cache: %s' % e)
    if cache:
        print('- update cache')
    else:
        print('- build cache from scratch')
        # get empty cache with distribution file
        distribution_file_data = _get_dist_file_data(index, dist_name, 'distribution')
        cache = DistributionCache(dist_name, distribution_file_data=distribution_file_data)

    # get distribution
    return get_cached_distribution(index, dist_name, cache=cache, allow_lazy_load=True), cache
