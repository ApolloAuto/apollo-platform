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

try:
    from urllib.request import urlopen
    from urllib.error import URLError
except ImportError:
    from urllib2 import urlopen
    from urllib2 import URLError

from rosdistro import logger
from rosdistro.manifest_provider import get_release_tag
from rosdistro.manifest_provider.git import check_remote_tag_exists


def github_manifest_provider(_dist_name, repo, pkg_name):
    assert repo.version
    if 'github.com' not in repo.url:
        logger.debug('Skip non-github url "%s"' % repo.url)
        raise RuntimeError('can not handle non github urls')

    release_tag = get_release_tag(repo, pkg_name)

    if not check_remote_tag_exists(repo.url, release_tag):
        raise RuntimeError('specified tag "%s" is not a git tag' % release_tag)

    url = repo.url
    if url.endswith('.git'):
        url = url[:-4]
    url += '/%s/package.xml' % release_tag
    if url.startswith('git://'):
        url = 'https://' + url[6:]
    if url.startswith('https://'):
        url = 'https://raw.' + url[8:]
    try:
        logger.debug('Load package.xml file from url "%s"' % url)
        package_xml = urlopen(url).read()
        return package_xml
    except URLError as e:
        logger.debug('- failed (%s), trying "%s"' % (e, url))
        raise RuntimeError()
