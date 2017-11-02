# Copyright (c) 2013, Open Source Robotics Foundation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author Paul Mathieu/paul@osrfoundation.org

import rosdistro


class PreRep137Warning(UserWarning):
    pass


class _RDCache:
    index_url = None
    index = None
    release_files = {}


class ReleaseFile(object):

    def __init__(self, dist_file):
        self.repositories = {}
        for repo_name in dist_file.repositories.keys():
            repo = dist_file.repositories[repo_name].release_repository
            if repo:
                self.repositories[repo_name] = repo
        self.platforms = dist_file.release_platforms


def _check_cache():
    if _RDCache.index_url != rosdistro.get_index_url():
        _RDCache.index_url = rosdistro.get_index_url()
        _RDCache.index = None
        _RDCache.release_files = {}


def get_index_url():
    _check_cache()
    return _RDCache.index_url


def get_index():
    _check_cache()
    if _RDCache.index is None:
        _RDCache.index = rosdistro.get_index(_RDCache.index_url)
    return _RDCache.index


def get_release_file(distro):
    _check_cache()
    if distro not in _RDCache.release_files:
        dist_file = rosdistro.get_distribution_file(get_index(), distro)
        _RDCache.release_files[distro] = ReleaseFile(dist_file)
    return _RDCache.release_files[distro]


def get_targets():
    return dict((d, get_release_file(d).platforms) for d in get_index().distributions)
