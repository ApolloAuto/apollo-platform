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

import socket
import time
try:
    from urllib.request import urlopen
    from urllib.error import HTTPError
    from urllib.error import URLError
except ImportError:
    from urllib2 import urlopen
    from urllib2 import HTTPError
    from urllib2 import URLError


def load_url(url, retry=2, retry_period=1, timeout=10, skip_decode=False):
    try:
        fh = urlopen(url, timeout=timeout)
    except HTTPError as e:
        if e.code == 503 and retry:
            time.sleep(retry_period)
            return load_url(url, retry=retry - 1, retry_period=retry_period, timeout=timeout)
        e.msg += ' (%s)' % url
        raise
    except URLError as e:
        if isinstance(e.reason, socket.timeout) and retry:
            time.sleep(retry_period)
            return load_url(url, retry=retry - 1, retry_period=retry_period, timeout=timeout)
        raise URLError(str(e) + ' (%s)' % url)
    # Python 2/3 Compatibility
    contents = fh.read()
    if isinstance(contents, str) or skip_decode:
        return contents
    else:
        return contents.decode('utf-8')
