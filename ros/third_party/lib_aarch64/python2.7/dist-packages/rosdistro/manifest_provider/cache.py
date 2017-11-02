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

from rosdistro import logger


class CachedManifestProvider(object):

    def __init__(self, distribution_cache, manifest_providers=None):
        self._distribution_cache = distribution_cache
        self._manifest_providers = manifest_providers

    def __call__(self, dist_name, repo, pkg_name):
        assert repo.version
        if pkg_name not in self._distribution_cache.release_package_xmls:
            # use manifest providers to lazy load
            package_xml = None
            for mp in self._manifest_providers or []:
                try:
                    package_xml = mp(dist_name, repo, pkg_name)
                    break
                except Exception as e:
                    # pass and try next manifest provider
                    logger.debug('Skipped "%s()": %s' % (mp.__name__, e))
            if package_xml is None:
                return None
            self._distribution_cache.release_package_xmls[pkg_name] = package_xml
        else:
            logger.debug('Load package.xml file for package "%s" from cache' % pkg_name)
        return self._distribution_cache.release_package_xmls[pkg_name]
